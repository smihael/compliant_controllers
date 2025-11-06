#pragma once

#include <array>
#include <string>
#include <atomic>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <std_msgs/msg/string.hpp> // retained only if future fallback needed

#include <compliant_controllers_msgs/msg/cartesian_command.hpp>
#include <compliant_controllers/robot_model.hpp>
#include <compliant_controllers/cartesian_impedance_impl.hpp>
#include <control/AbstractController.hpp> // Requires implementations define static constexpr kName


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace compliant_controllers {

class GenericCartesianControllerWrapper : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:

  std::string arm_id_;
  int num_joints_{7};
  double init_k_pos_{};
  double init_k_ori_{};

  // Robot description retrieval (via parameters client)
  std::string robot_description_node_{"robot_state_publisher"};
  std::string robot_description_param_{"robot_description"};
  std::string urdf_xml_;
  std::atomic<bool> urdf_received_{false};

  // Implementation loaded from external shared library impl_library_
  control::AbstractController* impl_{nullptr};
  compliant_controllers::RobotModel robot_model_;
  std::string impl_library_; // optional path to .so providing C factory interface
  // Dynamic loading handles
  void* impl_handle_{nullptr};
  using CreateFn = control::AbstractController*();
  using DestroyFn = void(control::AbstractController*);
  using NameFn = const char*();
  DestroyFn* destroy_fn_{nullptr};
  NameFn* name_fn_{nullptr};
  control::ControlCommand last_command_;
  bool have_last_command_{false};
  
  bool instantiateImplementation(std::string& err); // loads external implementation
  void destroyImplementation();

  
  Eigen::VectorXd tau_out_; // preallocated control output buffer
  control::ControllerState state_buffer_;

  // Nullspace desired defaults (kept for initial seed)
  Eigen::Matrix<double,7,1> q_d_nullspace_{Eigen::Matrix<double,7,1>::Zero()};
  Eigen::Matrix<double,7,1> nullspace_stiffness_{Eigen::Matrix<double,7,1>::Zero()};
  const double TAU_NULLSPACE_MAX_{5.0};

  realtime_tools::RealtimeBuffer<control::ControlCommand> rt_cartesian_cmd_buffer_;
  rclcpp::Subscription<compliant_controllers_msgs::msg::CartesianCommand>::SharedPtr cartesian_command_sub_;
  void cartesian_command_callback(const compliant_controllers_msgs::msg::CartesianCommand::SharedPtr msg);

  // Internal helper: update q,dq,tau from state interfaces and return aggregated ControllerState
  void readControllerState(control::ControllerState& out);

};

} // namespace compliant_controllers
