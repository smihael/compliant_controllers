#pragma once

#include <array>
#include <string>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <std_msgs/msg/string.hpp> // retained only if future fallback needed
#include <std_msgs/msg/int32.hpp>

#include <compliant_controllers_msgs/msg/cartesian_command.hpp>
#include <compliant_controllers/async_diagnostic_logger.hpp>
#include <compliant_controllers/friction_compensation.hpp>
#include <ros2_control_robot_dynamics/robot_description_loader.hpp>
#include <ros2_control_robot_dynamics/robot_model.hpp>
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
  struct GravityCompensation {
    bool enabled{false};
    Eigen::VectorXd tau;
  };

  struct EndEffectorLoadCompensation {
    bool enabled{false};
    double mass{0.0};
    double gravity_acceleration{9.80665};
    Eigen::Vector3d center_of_mass{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    Eigen::Matrix<double, 6, 1> wrench{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::VectorXd tau;
  };

  std::string arm_id_;
  int num_joints_{7};
  double init_k_pos_{};
  double init_k_ori_{};
  
  // Joint names (supports both Franka-style arm_id_joint{i} and UR-style named joints)
  std::vector<std::string> joint_names_;
  bool use_named_joints_{false};
  
  GravityCompensation gravity_compensation_;
  FrictionCompensation friction_compensation_;
  EndEffectorLoadCompensation end_effector_load_compensation_;
  AsyncDiagnosticLogger diagnostic_logger_;
  Eigen::VectorXd diagnostic_gravity_;
  Eigen::VectorXd diagnostic_coriolis_;
  Eigen::MatrixXd diagnostic_inertia_;

  std::string end_effector_profile_node_{};
  RobotDescriptionLoader robot_description_loader_;

  // Implementation loaded from external shared library impl_library_
  control::AbstractController* impl_{nullptr};
  compliant_controllers::RobotModel robot_model_;
  std::string impl_library_; // optional path to .so providing C factory interface
  // Dynamic loading handles
  void* impl_handle_{nullptr};
  using CreateFn = control::AbstractController*(int);
  using DestroyFn = void(control::AbstractController*);
  using NameFn = const char*();
  DestroyFn* destroy_fn_{nullptr};
  NameFn* name_fn_{nullptr};
  control::ControlCommand last_command_;
  bool have_last_command_{false};
  rclcpp::Time active_since_{0, 0, RCL_ROS_TIME};
  bool first_update_logged_{false};
  size_t debug_tick_{0};
  
  bool instantiateImplementation(std::string& err); // loads external implementation
  void destroyImplementation();
  void configureCompensationBuffers();
  void readEndEffectorLoadParameters();
  bool fetchEndEffectorLoadParametersFromNode(const std::string& node_name);
  void addGravityCompensation();
  void addFrictionCompensation(double dt);
  void addEndEffectorLoadCompensation();
  void sanitizeTorqueOutput();
  void writeTorqueOutput();
  void writeZeroTorques();
  void updateDiagnosticModelTerms();
  void logConfigurationSummary();
  void logActivationSummary();
  void logFirstUpdateSummary();

  
  Eigen::VectorXd tau_out_; // preallocated control output buffer
  control::ControllerState state_buffer_;

  // Nullspace desired defaults (kept for initial seed) - dynamically sized
  Eigen::VectorXd q_d_nullspace_;
  Eigen::VectorXd nullspace_stiffness_;
  const double TAU_NULLSPACE_MAX_{5.0};

  realtime_tools::RealtimeBuffer<control::ControlCommand> rt_cartesian_cmd_buffer_;
  rclcpp::Subscription<compliant_controllers_msgs::msg::CartesianCommand>::SharedPtr cartesian_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr diagnostic_mode_sub_;
  void cartesian_command_callback(const compliant_controllers_msgs::msg::CartesianCommand::SharedPtr msg);
  void diagnostic_mode_callback(const std_msgs::msg::Int32::SharedPtr msg);

  // Internal helper: update q,dq,tau from state interfaces and return aggregated ControllerState
  void readControllerState(control::ControllerState& out);

};

} // namespace compliant_controllers
