#include <compliant_controllers/friction_compensation.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace compliant_controllers {

class FrictionCompensation::Model {
public:
  virtual ~Model() = default;
  virtual const char* name() const = 0;
  virtual bool compute(const Eigen::VectorXd& q,
                       const Eigen::VectorXd& dq,
                       double dt,
                       Eigen::Ref<Eigen::VectorXd> tau) = 0;
};

namespace {

bool finite_vector(const std::vector<double>& values, int size) {
  return values.size() == static_cast<std::size_t>(size) &&
         std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

class SigmoidModel final : public FrictionCompensation::Model {
public:
  SigmoidModel(const std::vector<double>& phi1,
               const std::vector<double>& phi2,
               const std::vector<double>& phi3)
      : phi1_(Eigen::Map<const Eigen::VectorXd>(phi1.data(), phi1.size())),
        phi2_(Eigen::Map<const Eigen::VectorXd>(phi2.data(), phi2.size())),
        phi3_(Eigen::Map<const Eigen::VectorXd>(phi3.data(), phi3.size())) {}

  const char* name() const override { return "sigmoid"; }

  bool compute(const Eigen::VectorXd&,
               const Eigen::VectorXd& dq,
               double,
               Eigen::Ref<Eigen::VectorXd> tau) override {
    if (dq.size() != phi1_.size() || tau.size() != phi1_.size() ||
        !dq.array().isFinite().all()) {
      return false;
    }
    for (Eigen::Index i = 0; i < dq.size(); ++i) {
      const double arg =
          std::clamp(-phi2_(i) * (dq(i) + phi3_(i)), -60.0, 60.0);
      const double zero_arg =
          std::clamp(-phi2_(i) * phi3_(i), -60.0, 60.0);
      tau(i) = phi1_(i) / (1.0 + std::exp(arg)) -
               phi1_(i) / (1.0 + std::exp(zero_arg));
    }
    return tau.array().isFinite().all();
  }

private:
  Eigen::VectorXd phi1_;
  Eigen::VectorXd phi2_;
  Eigen::VectorXd phi3_;
};

class HysteresisModel final : public FrictionCompensation::Model {
public:
  HysteresisModel(const std::vector<double>& hyst,
                  const std::vector<double>& fvp,
                  const std::vector<double>& fcp,
                  const std::vector<double>& fvn,
                  const std::vector<double>& fcn)
      : hyst_(Eigen::Map<const Eigen::VectorXd>(hyst.data(), hyst.size())),
        fvp_(Eigen::Map<const Eigen::VectorXd>(fvp.data(), fvp.size())),
        fcp_(Eigen::Map<const Eigen::VectorXd>(fcp.data(), fcp.size())),
        fvn_(Eigen::Map<const Eigen::VectorXd>(fvn.data(), fvn.size())),
        fcn_(Eigen::Map<const Eigen::VectorXd>(fcn.data(), fcn.size())) {}

  const char* name() const override { return "hysteresis"; }

  bool compute(const Eigen::VectorXd&,
               const Eigen::VectorXd& dq,
               double,
               Eigen::Ref<Eigen::VectorXd> tau) override {
    if (dq.size() != hyst_.size() || tau.size() != hyst_.size() ||
        !dq.array().isFinite().all()) {
      return false;
    }
    tau.setZero();
    for (Eigen::Index i = 0; i < dq.size(); ++i) {
      if (dq(i) >= hyst_(i)) {
        tau(i) = fvp_(i) * dq(i) + fcp_(i);
      } else if (dq(i) <= -hyst_(i)) {
        tau(i) = fvn_(i) * dq(i) - fcn_(i);
      }
    }
    return tau.array().isFinite().all();
  }

private:
  Eigen::VectorXd hyst_;
  Eigen::VectorXd fvp_;
  Eigen::VectorXd fcp_;
  Eigen::VectorXd fvn_;
  Eigen::VectorXd fcn_;
};

}  // namespace

FrictionCompensation::FrictionCompensation() = default;
FrictionCompensation::~FrictionCompensation() = default;

bool FrictionCompensation::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    int num_joints,
    bool log_updates) {
  enabled_ = false;
  model_name_ = "disabled";
  scale_ = 1.0;
  num_joints_ = num_joints;
  model_.reset();

  if (num_joints_ <= 0) {
    RCLCPP_ERROR(node->get_logger(),
                 "Friction compensation requires a positive joint count.");
    return false;
  }
  tau_.setZero(num_joints_);
  empty_q_.setZero(num_joints_);

  bool requested = false;
  node->get_parameter("friction_compensation_enabled", requested);
  if (!requested) {
    if (log_updates) {
      RCLCPP_INFO(node->get_logger(), "Friction compensation: disabled");
    }
    return true;
  }

  std::string requested_model;
  if (!node->get_parameter("friction_compensation.model", requested_model) ||
      requested_model.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "friction_compensation.model is required when compensation is enabled.");
    return false;
  }
  node->get_parameter("friction_compensation.scale", scale_);
  if (!std::isfinite(scale_) || scale_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(),
                 "friction_compensation.scale must be finite and non-negative.");
    return false;
  }

  auto get_vector = [&node](const char* suffix, std::vector<double>& values) {
    return node->get_parameter(
        std::string("friction_compensation.") + suffix, values);
  };

  if (requested_model == "sigmoid") {
    std::vector<double> phi1, phi2, phi3;
    if (!get_vector("phi1", phi1) || !get_vector("phi2", phi2) ||
        !get_vector("phi3", phi3) || !finite_vector(phi1, num_joints_) ||
        !finite_vector(phi2, num_joints_) || !finite_vector(phi3, num_joints_)) {
      RCLCPP_ERROR(node->get_logger(),
                   "Sigmoid model requires finite phi1, phi2, and phi3 vectors "
                   "with exactly %d entries.", num_joints_);
      return false;
    }
    model_ = std::make_unique<SigmoidModel>(phi1, phi2, phi3);
  } else if (requested_model == "hysteresis") {
    std::vector<double> hyst, fvp, fcp, fvn, fcn;
    if (!get_vector("hyst", hyst) || !get_vector("fvp", fvp) ||
        !get_vector("fcp", fcp) || !get_vector("fvn", fvn) ||
        !get_vector("fcn", fcn) || !finite_vector(hyst, num_joints_) ||
        !finite_vector(fvp, num_joints_) || !finite_vector(fcp, num_joints_) ||
        !finite_vector(fvn, num_joints_) || !finite_vector(fcn, num_joints_) ||
        std::any_of(hyst.begin(), hyst.end(),
                    [](double value) { return value < 0.0; })) {
      RCLCPP_ERROR(node->get_logger(),
                   "Hysteresis model requires finite hyst, fvp, fcp, fvn, and "
                   "fcn vectors with exactly %d entries; hyst must be non-negative.",
                   num_joints_);
      return false;
    }
    model_ = std::make_unique<HysteresisModel>(hyst, fvp, fcp, fvn, fcn);
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "Unsupported friction compensation model '%s'; expected "
                 "'sigmoid' or 'hysteresis'.", requested_model.c_str());
    return false;
  }

  model_name_ = model_->name();
  enabled_ = true;
  if (log_updates) {
    RCLCPP_INFO(node->get_logger(),
                "Friction compensation: enabled model=%s scale=%.3f",
                model_name_.c_str(), scale_);
  }
  return true;
}

bool FrictionCompensation::add(const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq,
                               double dt,
                               Eigen::Ref<Eigen::VectorXd> tau_out) {
  if (!enabled_ || scale_ == 0.0) {
    return true;
  }
  if (!model_ || q.size() != num_joints_ || dq.size() != num_joints_ ||
      tau_out.size() != num_joints_ || !q.array().isFinite().all() ||
      !dq.array().isFinite().all() || !tau_out.array().isFinite().all() ||
      !std::isfinite(dt)) {
    enabled_ = false;
    return false;
  }
  tau_.setZero();
  if (!model_->compute(q, dq, dt, tau_) || !tau_.array().isFinite().all()) {
    enabled_ = false;
    return false;
  }
  tau_out += scale_ * tau_;
  return tau_out.array().isFinite().all();
}

}  // namespace compliant_controllers
