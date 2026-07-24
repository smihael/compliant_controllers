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

#include <robot_module_msgs/msg/cartesian_command.hpp>

#include <compliant_controllers/robot_model.hpp>


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

  std::string arm_id_;
  int num_joints_{7};
  std::array<double,7> k_p_{};
  std::array<double,7> k_d_{};
  std::array<double,7> last_position_{};
  std::array<double,7> last_velocity_{};
  bool first_update_{true};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;

  // Robot description retrieval (via parameters client)
  std::string robot_description_node_{"robot_state_publisher"};
  std::string robot_description_param_{"robot_description"};
  std::string urdf_xml_;
  std::atomic<bool> urdf_received_{false};

  // Robot model wrapper (Pinocchio hidden)
  RobotModel robot_model_; // handles building & kinematics

  // Joint state vectors (Eigen)
  using Vector7d = Eigen::Matrix<double, 7, 1>; 
  Vector7d q, dq, tau;

  std::vector<const hardware_interface::LoanedStateInterface*> position_interfaces_;
  std::vector<const hardware_interface::LoanedStateInterface*> velocity_interfaces_;
  std::vector<const hardware_interface::LoanedStateInterface*> effort_interfaces_;

  // Desired Cartesian state
  Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double,6,1> velocity_d_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> wrench_d_{Eigen::Matrix<double,6,1>::Zero()};

  // Jacobian buffer
  Eigen::Matrix<double,6,Eigen::Dynamic> J_;

  // Stiffness & damping (6x6)
  Eigen::Matrix<double,6,6> cartesian_stiffness_{Eigen::Matrix<double,6,6>::Zero()};
  Eigen::Matrix<double,6,6> cartesian_damping_{Eigen::Matrix<double,6,6>::Zero()};

  // Nullspace control
  Eigen::Matrix<double,7,1> q_d_nullspace_{Eigen::Matrix<double,7,1>::Zero()};
  Eigen::Matrix<double,7,1> nullspace_stiffness_{Eigen::Matrix<double,7,1>::Zero()};
  const double TAU_NULLSPACE_MAX_{5.0};

  // Realtime Cartesian command handling
  struct CartesianCommandRT {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double,6,1> velocity;
    Eigen::Matrix<double,6,1> wrench;
    Eigen::Matrix<double,6,6> stiffness;
    Eigen::Matrix<double,6,6> damping;
  // Optional nullspace and torque feedforward (sized to controller joints)
  Eigen::VectorXd q_ns_des;     // size = num_joints_
  Eigen::VectorXd k_ns;         // size = num_joints_
  Eigen::VectorXd d_ns;         // size = num_joints_
  Eigen::VectorXd tau_ff;       // size = num_joints_
  bool has_nullspace{false};
  bool has_tau_ff{false};
    bool has_value{false};
  };
  realtime_tools::RealtimeBuffer<CartesianCommandRT> rt_cartesian_cmd_buffer_;
  rclcpp::Subscription<robot_module_msgs::msg::CartesianCommand>::SharedPtr cartesian_command_sub_;
  void cartesian_command_callback(const robot_module_msgs::msg::CartesianCommand::SharedPtr msg);

  // Helper functions
  Eigen::Vector3d q_log(const Eigen::Quaterniond &q) const; // quaternion log map
  void pseudoInverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_pinv, double tolerance = 1e-6) const;
};

} // namespace compliant_controllers
