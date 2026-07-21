#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <compliant_controllers/generic_controller_wrapper.hpp>
#include <compliant_controllers_msgs/msg/joint_command.hpp>
#include <control/AbstractController.hpp>
#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace compliant_controllers {

class GenericJointControllerWrapper : public GenericControllerWrapper {
public:
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  bool use_named_joints_{false};

  bool add_friction_compensation_{false};

  Eigen::VectorXd init_stiffness_;
  Eigen::VectorXd init_damping_;

  realtime_tools::RealtimeBuffer<control::ControlCommand> rt_joint_cmd_buffer_;
  rclcpp::Subscription<compliant_controllers_msgs::msg::JointCommand>::SharedPtr joint_command_sub_;

  void joint_command_callback(const compliant_controllers_msgs::msg::JointCommand::SharedPtr msg);
  void logConfigurationSummary();
  void logActivationSummary();
  void logFirstUpdateSummary();
};

}  // namespace compliant_controllers
