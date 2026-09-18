#pragma once

#include <array>
#include <mutex>
#include <numbers>
#include <string>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <compliant_controllers/generic_controller_wrapper.hpp>
#include <compliant_controllers/cartesian_error_biased_dither.hpp>
#include <compliant_controllers_msgs/msg/cartesian_command.hpp>
#include <control/AbstractController.hpp> // Requires implementations define static constexpr kName


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace compliant_controllers {

class GenericCartesianControllerWrapper : public GenericControllerWrapper {
public:
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  double init_k_pos_{};
  double init_k_ori_{};
  bool max_step_guard_enabled_{true};
  double max_position_command_step_m_{0.01};
  double max_orientation_command_step_rad_{1.0 * std::numbers::pi / 180.0};

  bool robot_gravity_compensation_enabled_{false};
  bool dithering_enabled_{false};
  CartesianErrorBiasedDither error_biased_dither_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> dither_jacobian_;
  Eigen::Matrix<double, 6, 1> dither_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::VectorXd dither_torque_;

  control::ControlCommand last_command_;
  bool have_last_command_{false};
  std::mutex command_mutex_;
  size_t debug_tick_{0};
  
  void addFrictionCompensation(double dt);
  void addErrorBiasedDither(const control::ControlCommand& command, double dt);
  void logConfigurationSummary();
  void logActivationSummary();
  void logFirstUpdateSummary();

  // Nullspace desired defaults (kept for initial seed) - dynamically sized
  Eigen::VectorXd q_d_nullspace_;
  Eigen::VectorXd nullspace_stiffness_;
  const double TAU_NULLSPACE_MAX_{5.0};

  realtime_tools::RealtimeBuffer<control::ControlCommand> rt_cartesian_cmd_buffer_;
  rclcpp::Subscription<compliant_controllers_msgs::msg::CartesianCommand>::SharedPtr cartesian_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_stiffness_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr orientation_stiffness_sub_;
  void cartesian_command_callback(const compliant_controllers_msgs::msg::CartesianCommand::SharedPtr msg);
  void position_stiffness_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void orientation_stiffness_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

};

} // namespace compliant_controllers
