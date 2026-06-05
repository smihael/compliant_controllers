#include <compliant_controllers/friction_compensation.hpp>
#include <compliant_controllers/parameter_utils.hpp>

#include "friction_compensation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace compliant_controllers {

namespace {
bool has_expected_size(const std::vector<double>& values, int expected) {
  return values.size() == static_cast<std::size_t>(expected);
}
}  // namespace

class FrictionCompensation::Model {
public:
  virtual ~Model() = default;
  virtual const char* name() const = 0;
  virtual bool compute(const Eigen::VectorXd& q,
                       const Eigen::VectorXd& dq,
                       double dt,
                       Eigen::Ref<Eigen::VectorXd> tau) = 0;
};

class SigmoidFrictionCompensation final : public FrictionCompensation::Model {
public:
  SigmoidFrictionCompensation(const std::vector<double>& phi1,
                              const std::vector<double>& phi2,
                              const std::vector<double>& phi3,
                              int num_joints)
      : phi1_(Eigen::Map<const Eigen::VectorXd>(phi1.data(), num_joints)),
        phi2_(Eigen::Map<const Eigen::VectorXd>(phi2.data(), num_joints)),
        phi3_(Eigen::Map<const Eigen::VectorXd>(phi3.data(), num_joints)),
        num_joints_(num_joints) {}

  const char* name() const override { return "sigmoid"; }

  bool compute(const Eigen::VectorXd& /*q*/,
               const Eigen::VectorXd& dq,
               double /*dt*/,
               Eigen::Ref<Eigen::VectorXd> tau) override {
    if (dq.size() != num_joints_ || tau.size() != num_joints_) {
      return false;
    }
    for (int i = 0; i < num_joints_; ++i) {
      const double arg = std::clamp(-phi2_(i) * (dq(i) + phi3_(i)), -60.0, 60.0);
      const double arg0 = std::clamp(-phi2_(i) * phi3_(i), -60.0, 60.0);
      tau(i) = phi1_(i) / (1.0 + std::exp(arg)) - phi1_(i) / (1.0 + std::exp(arg0));
    }
    return tau.array().isFinite().all();
  }

private:
  Eigen::VectorXd phi1_;
  Eigen::VectorXd phi2_;
  Eigen::VectorXd phi3_;
  int num_joints_{0};
};

class HysteresisFrictionCompensation final : public FrictionCompensation::Model {
public:
  HysteresisFrictionCompensation(const std::vector<double>& hyst,
                                 const std::vector<double>& fvp,
                                 const std::vector<double>& fcp,
                                 const std::vector<double>& fvn,
                                 const std::vector<double>& fcn,
                                 int num_joints)
      : hyst_(Eigen::Map<const Eigen::VectorXd>(hyst.data(), num_joints)),
        fvp_(Eigen::Map<const Eigen::VectorXd>(fvp.data(), num_joints)),
        fcp_(Eigen::Map<const Eigen::VectorXd>(fcp.data(), num_joints)),
        fvn_(Eigen::Map<const Eigen::VectorXd>(fvn.data(), num_joints)),
        fcn_(Eigen::Map<const Eigen::VectorXd>(fcn.data(), num_joints)),
        num_joints_(num_joints) {}

  const char* name() const override { return "hysteresis"; }

  bool compute(const Eigen::VectorXd& /*q*/,
               const Eigen::VectorXd& dq,
               double /*dt*/,
               Eigen::Ref<Eigen::VectorXd> tau) override {
    if (dq.size() != num_joints_ || tau.size() != num_joints_) {
      return false;
    }
    for (int i = 0; i < num_joints_; ++i) {
      tau(i) = 0.0;
      if (dq(i) >= hyst_(i)) {
        tau(i) = fvp_(i) * dq(i) + fcp_(i);
      }
      if (dq(i) <= -hyst_(i)) {
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
  int num_joints_{0};
};

class FcijsFrictionCompensation final : public FrictionCompensation::Model {
public:
  explicit FcijsFrictionCompensation(int num_joints, bool use_gating)
      : num_joints_(num_joints) {
    for (int i = 0; i < friction_compensation::FrictionModelParams::kNumJoints; ++i) {
      params_.joints[i].use_gating = use_gating;
    }
    compensators_ = std::make_unique<CompensatorArray>(makeCompensators(params_));
  }

  const char* name() const override { return "fcijs"; }

  bool compute(const Eigen::VectorXd& q,
               const Eigen::VectorXd& dq,
               double dt,
               Eigen::Ref<Eigen::VectorXd> tau) override {
    if (num_joints_ != friction_compensation::FrictionModelParams::kNumJoints ||
        q.size() != num_joints_ || dq.size() != num_joints_ || tau.size() != num_joints_) {
      return false;
    }

    if (!initialized_) {
      const double init_dt = dt > 0.0 ? dt : 1e-3;
      for (int i = 0; i < num_joints_; ++i) {
        (*compensators_)[i].Reset();
        (*compensators_)[i].Initialize(q(i), dq(i), init_dt);
      }
      initialized_ = true;
    }

    const double step_dt = dt > 0.0 ? dt : 1e-3;
    for (int i = 0; i < num_joints_; ++i) {
      tau(i) = (*compensators_)[i].Step(step_dt, q(i), dq(i));
    }
    return tau.array().isFinite().all();
  }

private:
  using CompensatorArray =
    std::array<friction_compensation::SingleJointFrictionCompensator,
               friction_compensation::FrictionModelParams::kNumJoints>;

  static CompensatorArray makeCompensators(const friction_compensation::FrictionModelParams& params) {
    return {{
      friction_compensation::SingleJointFrictionCompensator(params.joints[0], friction_compensation::kDataDefault.joints[0]),
      friction_compensation::SingleJointFrictionCompensator(params.joints[1], friction_compensation::kDataDefault.joints[1]),
      friction_compensation::SingleJointFrictionCompensator(params.joints[2], friction_compensation::kDataDefault.joints[2]),
      friction_compensation::SingleJointFrictionCompensator(params.joints[3], friction_compensation::kDataDefault.joints[3]),
      friction_compensation::SingleJointFrictionCompensator(params.joints[4], friction_compensation::kDataDefault.joints[4]),
      friction_compensation::SingleJointFrictionCompensator(params.joints[5], friction_compensation::kDataDefault.joints[5]),
      friction_compensation::SingleJointFrictionCompensator(params.joints[6], friction_compensation::kDataDefault.joints[6]),
    }};
  }

  int num_joints_{0};
  bool initialized_{false};
  friction_compensation::FrictionModelParams params_{};
  std::unique_ptr<CompensatorArray> compensators_;
};

FrictionCompensation::FrictionCompensation() = default;
FrictionCompensation::~FrictionCompensation() = default;

bool FrictionCompensation::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     int num_joints,
                                     bool log_updates) {
  enabled_ = false;
  model_.reset();
  model_name_ = "disabled";
  num_joints_ = num_joints;
  tau_.setZero(num_joints_);
  empty_q_.resize(0);

  const bool requested = parameter_utils::get_optional_bool(node, "add_friction_compensation", false);
  const auto phi1 = parameter_utils::get_optional_double_array(node, "friction_compensation.phi1");
  const auto phi2 = parameter_utils::get_optional_double_array(node, "friction_compensation.phi2");
  const auto phi3 = parameter_utils::get_optional_double_array(node, "friction_compensation.phi3");
  const auto hyst = parameter_utils::get_optional_double_array(node, "friction_compensation.hyst");
  const auto fvp = parameter_utils::get_optional_double_array(node, "friction_compensation.fvp");
  const auto fcp = parameter_utils::get_optional_double_array(node, "friction_compensation.fcp");
  const auto fvn = parameter_utils::get_optional_double_array(node, "friction_compensation.fvn");
  const auto fcn = parameter_utils::get_optional_double_array(node, "friction_compensation.fcn");
  const auto requested_model = parameter_utils::get_optional_string(node, "friction_compensation.model", "auto");
  const bool use_gating = parameter_utils::get_optional_bool(node, "friction_compensation.use_gating", true);
  scale_ = parameter_utils::get_optional_double(node, "friction_compensation.scale", 1.0);

  if (!std::isfinite(scale_) || scale_ < 0.0) {
    if (log_updates) {
      RCLCPP_WARN(node->get_logger(),
                  "Invalid friction_compensation.scale=%.6f. Falling back to 1.0.",
                  scale_);
    }
    scale_ = 1.0;
  }

  const bool params_present =
      !phi1.empty() || !phi2.empty() || !phi3.empty() || !hyst.empty() || !fvp.empty() ||
      !fcp.empty() || !fvn.empty() || !fcn.empty();
  if (!(requested || params_present)) {
    return true;
  }

  const bool has_hysteresis =
      has_expected_size(hyst, num_joints_) && has_expected_size(fvp, num_joints_) &&
      has_expected_size(fcp, num_joints_) && has_expected_size(fvn, num_joints_) &&
      has_expected_size(fcn, num_joints_);
  const bool has_sigmoid =
      has_expected_size(phi1, num_joints_) && has_expected_size(phi2, num_joints_) &&
      has_expected_size(phi3, num_joints_);

  if (requested_model == "fcijs") {
    model_ = std::make_unique<FcijsFrictionCompensation>(num_joints_, use_gating);
  } else if ((requested_model == "hysteresis" || (requested_model == "auto" && has_hysteresis)) &&
      has_hysteresis) {
    model_ = std::make_unique<HysteresisFrictionCompensation>(hyst, fvp, fcp, fvn, fcn, num_joints_);
  } else if ((requested_model == "sigmoid" || requested_model == "auto") && has_sigmoid) {
    model_ = std::make_unique<SigmoidFrictionCompensation>(phi1, phi2, phi3, num_joints_);
  } else {
    if (log_updates) {
      RCLCPP_WARN(node->get_logger(),
                  "Friction compensation requested but parameters for model '%s' are invalid (expected %d values per joint). Disabled.",
                  requested_model.c_str(), num_joints_);
    }
    return true;
  }

  enabled_ = true;
  model_name_ = model_->name();
  if (log_updates) {
    RCLCPP_INFO(node->get_logger(),
                "Friction compensation (%s): enabled with %d-joint parameter vectors (scale=%.3f).",
                model_name_.c_str(), num_joints_, scale_);
  }
  return true;
}

bool FrictionCompensation::add(const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq,
                               double dt,
                               Eigen::Ref<Eigen::VectorXd> tau_out) {
  if (!enabled_) {
    return true;
  }
  if (model_ == nullptr || tau_out.size() != num_joints_ || tau_.size() != num_joints_) {
    enabled_ = false;
    return false;
  }
  if (scale_ <= 0.0) {
    return true;
  }

  tau_.setZero();
  if (!model_->compute(q, dq, dt, tau_)) {
    enabled_ = false;
    return false;
  }
  tau_out += scale_ * tau_;
  return true;
}

}  // namespace compliant_controllers
