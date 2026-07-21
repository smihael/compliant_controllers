#include <compliant_controllers/friction_compensation.hpp>

#include <algorithm>
#include <array>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace compliant_controllers
{

class FrictionCompensation::Model
{
public:
  virtual ~Model() = default;

  virtual const char* name() const = 0;
  virtual bool compute(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, double dt,
                       Eigen::Ref<Eigen::VectorXd> tau) = 0;
};

namespace
{

constexpr int kDeLucaJointCount = 7;

// Parameters based on Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization
constexpr std::array<double, kDeLucaJointCount> kDeLucaPhi1{ -1.815001380, -5.000000000, -1.193582344, -5.000000000, -1.096575841, -1.297481993, -0.595999358 };
constexpr std::array<double, kDeLucaJointCount> kDeLucaPhi2{ 30.000000000, 30.000000000, 22.579578200, 30.000000000, 30.000000000, 30.000000000, 22.099240539 };
constexpr std::array<double, kDeLucaJointCount> kDeLucaPhi3{ -0.002642619, -0.115656453, 0.000297366, 0.096129698, 0.002909211,  0.134123947,  0.002213736 };

class DeLucaFrictionCompensation final : public FrictionCompensation::Model
{
public:
  const char* name() const override
  {
    return "de_luca";
  }

  bool compute(const Eigen::VectorXd& /*q*/, const Eigen::VectorXd& dq, double /*dt*/,
               Eigen::Ref<Eigen::VectorXd> tau) override
  {
    if (dq.size() != kDeLucaJointCount || tau.size() != kDeLucaJointCount)
    {
      return false;
    }

    for (int i = 0; i < kDeLucaJointCount; ++i)
    {
      const double arg = std::clamp(-kDeLucaPhi2[i] * (dq(i) + kDeLucaPhi3[i]), -60.0, 60.0);
      const double arg_at_zero = std::clamp(-kDeLucaPhi2[i] * kDeLucaPhi3[i], -60.0, 60.0);
      tau(i) = kDeLucaPhi1[i] / (1.0 + std::exp(arg)) - kDeLucaPhi1[i] / (1.0 + std::exp(arg_at_zero));
    }
    return tau.array().isFinite().all();
  }
};

class DummyFrictionCompensation final : public FrictionCompensation::Model
{
public:
  const char* name() const override
  {
    return "dummy";
  }

  bool compute(const Eigen::VectorXd& /*q*/, const Eigen::VectorXd& /*dq*/, double /*dt*/,
               Eigen::Ref<Eigen::VectorXd> tau) override
  {
    tau.setZero();
    return true;
  }
};

}  // namespace

FrictionCompensation::FrictionCompensation() = default;
FrictionCompensation::~FrictionCompensation() = default;

bool FrictionCompensation::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, int num_joints,
                                     bool log_updates)
{
  enabled_ = false;
  model_name_ = "disabled";
  scale_ = 1.0;
  num_joints_ = num_joints;
  model_.reset();

  if (num_joints_ <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Friction compensation requires a positive joint count, got %d.", num_joints_);
    return false;
  }

  tau_.setZero(num_joints_);
  empty_q_.setZero(num_joints_);

  bool requested = false;
  node->get_parameter("friction_compensation_enabled", requested);
  if (!requested)
  {
    if (log_updates)
    {
      RCLCPP_INFO(node->get_logger(), "Friction compensation: disabled");
    }
    return true;
  }

  std::string requested_model{ "auto" };
  node->get_parameter("friction_compensation.model", requested_model);
  node->get_parameter("friction_compensation.scale", scale_);

  if (!std::isfinite(scale_) || scale_ < 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "friction_compensation.scale must be finite, got %.6f.", scale_);
    return false;
  }

  if (requested_model.empty() || requested_model == "auto" || requested_model == "sigmoid")
  {
    requested_model = "de_luca";
  }

  if (requested_model == "de_luca")
  {
    if (num_joints_ != kDeLucaJointCount)
    {
      RCLCPP_ERROR(node->get_logger(), "De Luca friction compensation requires %d joints, got %d.", kDeLucaJointCount,
                   num_joints_);
      return false;
    }
    model_ = std::make_unique<DeLucaFrictionCompensation>();
  }
  else if (requested_model == "dummy")
  {
    model_ = std::make_unique<DummyFrictionCompensation>();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Unknown friction compensation model '%s'.", requested_model.c_str());
    return false;
  }

  model_name_ = model_->name();
  enabled_ = true;

  if (log_updates)
  {
    RCLCPP_INFO(node->get_logger(), "Friction compensation: enabled model=%s scale=%.3f", model_name_.c_str(), scale_);
  }

  return true;
}

bool FrictionCompensation::add(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, double dt,
                               Eigen::Ref<Eigen::VectorXd> tau_out)
{
  if (!enabled_)
  {
    return true;
  }

  if (!model_)
  {
    enabled_ = false;
    return false;
  }

  if (q.size() != num_joints_ || dq.size() != num_joints_ || tau_out.size() != num_joints_)
  {
    enabled_ = false;
    return false;
  }

  if (!std::isfinite(dt))
  {
    enabled_ = false;
    return false;
  }

  tau_.setZero();
  if (!model_->compute(q, dq, dt, tau_) || !tau_.array().isFinite().all())
  {
    enabled_ = false;
    return false;
  }

  tau_out += scale_ * tau_;
  return true;
}

}  // namespace compliant_controllers
