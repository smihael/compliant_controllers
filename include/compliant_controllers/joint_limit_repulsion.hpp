#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <urdf/model.h>

namespace compliant_controllers {

// Adds a virtual spring-damper torque near the position limits stored in the URDF.
// Configuration is performed outside the real-time loop; addTorque() allocates no memory.
class JointLimitRepulsion {
public:
  bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 const std::string& urdf_xml,
                 const std::vector<std::string>& joint_names) {
    enabled_ = false;
    if (node->has_parameter("joint_limit_repulsion.enabled")) {
      node->get_parameter("joint_limit_repulsion.enabled", enabled_);
    }

    const auto size = static_cast<Eigen::Index>(joint_names.size());
    lower_.setConstant(size, -std::numeric_limits<double>::infinity());
    upper_.setConstant(size, std::numeric_limits<double>::infinity());
    stiffness_.setZero(size);
    damping_.setZero(size);
    margin_.setZero(size);
    has_position_limit_.assign(joint_names.size(), false);

    if (!enabled_) {
      return true;
    }

    if (!readVectorParameter(node, "joint_limit_repulsion.stiffness", size, stiffness_) ||
        !readVectorParameter(node, "joint_limit_repulsion.damping", size, damping_) ||
        !readVectorParameter(node, "joint_limit_repulsion.margin", size, margin_)) {
      RCLCPP_ERROR(node->get_logger(),
                   "Joint limit repulsion parameters must contain one value or one value per controlled joint.");
      return false;
    }
    if (!(stiffness_.array().isFinite().all() && (stiffness_.array() >= 0.0).all()) ||
        !(damping_.array().isFinite().all() && (damping_.array() >= 0.0).all()) ||
        !(margin_.array().isFinite().all() && (margin_.array() > 0.0).all())) {
      RCLCPP_ERROR(node->get_logger(),
                   "Joint limit repulsion stiffness/damping must be finite and non-negative, and margin must be finite and positive.");
      return false;
    }

    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF while configuring joint limit repulsion.");
      return false;
    }

    for (Eigen::Index i = 0; i < size; ++i) {
      const auto joint = model.getJoint(joint_names[static_cast<size_t>(i)]);
      if (!joint) {
        RCLCPP_ERROR(node->get_logger(), "Controlled joint '%s' is missing from the URDF.",
                     joint_names[static_cast<size_t>(i)].c_str());
        return false;
      }
      if (joint->type == urdf::Joint::CONTINUOUS || !joint->limits) {
        RCLCPP_WARN(node->get_logger(),
                    "Joint '%s' has no finite position limits; joint limit repulsion is disabled for it.",
                    joint_names[static_cast<size_t>(i)].c_str());
        continue;
      }

      lower_(i) = joint->limits->lower;
      upper_(i) = joint->limits->upper;
      if (!std::isfinite(lower_(i)) || !std::isfinite(upper_(i)) || lower_(i) >= upper_(i)) {
        RCLCPP_ERROR(node->get_logger(), "Joint '%s' has invalid URDF position limits [%g, %g].",
                     joint_names[static_cast<size_t>(i)].c_str(), lower_(i), upper_(i));
        return false;
      }
      if (2.0 * margin_(i) >= upper_(i) - lower_(i)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Joint limit repulsion margin for '%s' must be less than half its URDF range.",
                     joint_names[static_cast<size_t>(i)].c_str());
        return false;
      }
      has_position_limit_[static_cast<size_t>(i)] = true;
    }

    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Joint limit repulsion enabled using URDF limits: stiffness=["
                         << stiffness_.transpose() << "], damping=[" << damping_.transpose()
                         << "], margin=[" << margin_.transpose() << "]");
    return true;
  }

  bool addTorque(const Eigen::Ref<const Eigen::VectorXd>& position,
                 const Eigen::Ref<const Eigen::VectorXd>& velocity,
                 Eigen::Ref<Eigen::VectorXd> torque) const noexcept {
    if (!enabled_) {
      return true;
    }
    if (position.size() != lower_.size() || velocity.size() != lower_.size() ||
        torque.size() != lower_.size()) {
      return false;
    }

    for (Eigen::Index i = 0; i < lower_.size(); ++i) {
      if (!has_position_limit_[static_cast<size_t>(i)]) {
        continue;
      }
      const double lower_activation = lower_(i) + margin_(i);
      const double upper_activation = upper_(i) - margin_(i);
      if (position(i) < lower_activation) {
        torque(i) += stiffness_(i) * (lower_activation - position(i)) - damping_(i) * velocity(i);
      } else if (position(i) > upper_activation) {
        torque(i) += -stiffness_(i) * (position(i) - upper_activation) - damping_(i) * velocity(i);
      }
    }
    return torque.array().isFinite().all();
  }

  bool enabled() const noexcept { return enabled_; }

private:
  static bool readVectorParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                  const std::string& name,
                                  Eigen::Index size,
                                  Eigen::VectorXd& output) {
    std::vector<double> values;
    if (!node->get_parameter(name, values) || values.empty()) {
      return false;
    }
    if (values.size() == 1) {
      output.setConstant(size, values.front());
      return true;
    }
    if (values.size() != static_cast<size_t>(size)) {
      return false;
    }
    output = Eigen::Map<const Eigen::VectorXd>(values.data(), size);
    return true;
  }

  bool enabled_{false};
  Eigen::VectorXd lower_;
  Eigen::VectorXd upper_;
  Eigen::VectorXd stiffness_;
  Eigen::VectorXd damping_;
  Eigen::VectorXd margin_;
  std::vector<bool> has_position_limit_;
};

}  // namespace compliant_controllers
