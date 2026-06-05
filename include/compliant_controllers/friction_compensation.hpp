#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace compliant_controllers {

class FrictionCompensation {
public:
  class Model;

  FrictionCompensation();
  ~FrictionCompensation();

  bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 int num_joints,
                 bool log_updates);

  bool enabled() const { return enabled_; }
  const std::string& modelName() const { return model_name_; }
  double scale() const { return scale_; }

  bool add(const Eigen::VectorXd& q,
           const Eigen::VectorXd& dq,
           double dt,
           Eigen::Ref<Eigen::VectorXd> tau_out);

  bool add(const Eigen::VectorXd& dq, Eigen::Ref<Eigen::VectorXd> tau_out) {
    return add(empty_q_, dq, 0.001, tau_out);
  }

private:
  bool enabled_{false};
  std::string model_name_{"disabled"};
  double scale_{1.0};
  int num_joints_{0};
  Eigen::VectorXd tau_;
  Eigen::VectorXd empty_q_;
  std::unique_ptr<Model> model_;
};

}  // namespace compliant_controllers
