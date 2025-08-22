#pragma once

#include <array>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace compliant_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  struct Command { std::array<double,7> q{}; bool has_value{false}; };
  realtime_tools::RealtimeBuffer<Command> command_buffer_;

  std::string arm_id_{"panda"};
  int num_joints_{7};
  std::array<double,7> k_p_{};
  std::array<double,7> k_d_{};
  std::array<double,7> last_position_{};
  std::array<double,7> last_velocity_{};
  bool first_update_{true};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
};

} // namespace compliant_controllers
