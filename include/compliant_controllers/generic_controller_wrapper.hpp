#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <compliant_controllers/async_diagnostic_logger.hpp>
#include <compliant_controllers/friction_compensation.hpp>
#include <compliant_controllers/joint_limit_repulsion.hpp>
#include <control/AbstractController.hpp>
#include <control/ControlStates.hpp>
#include <ros2_control_robot_dynamics/robot_description_loader.hpp>
#include <ros2_control_robot_dynamics/robot_model.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace compliant_controllers {

class GenericControllerWrapper : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  using DestroyFn = void(control::AbstractController*);
  using NameFn = const char*();

  void configureLogging(const std::string& wrapper_name);
  bool instantiateImplementation(std::string& err);
  void destroyImplementation();
  void diagnostic_log_filter_tag_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void readControllerState(control::ControllerState& out);
  void sanitizeTorqueOutput();
  void writeTorqueOutput();
  void writeZeroTorques();

  std::string arm_id_;
  rclcpp::Logger logger_{rclcpp::get_logger("GenericControllerWrapper")};
  int num_joints_{7};
  std::vector<std::string> joint_names_;

  RobotDescriptionLoader robot_description_loader_;
  RobotModel robot_model_;
  FrictionCompensation friction_compensation_;
  JointLimitRepulsion joint_limit_repulsion_;
  AsyncDiagnosticLogger diagnostic_logger_;

  std::string impl_library_;
  control::AbstractController* impl_{nullptr};
  void* impl_handle_{nullptr};
  DestroyFn* destroy_fn_{nullptr};
  NameFn* name_fn_{nullptr};

  control::ControllerState state_buffer_;
  Eigen::VectorXd tau_out_;
  rclcpp::Time active_since_{0, 0, RCL_ROS_TIME};
  bool first_update_logged_{false};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr diagnostic_log_filter_tag_sub_;

private:
  struct LoadedControllerLibrary {
    void* handle{nullptr};
    control::AbstractController* impl{nullptr};
    DestroyFn* destroy_fn{nullptr};
    NameFn* name_fn{nullptr};
  };

  std::string resolveSharedLibraryPath(const std::string& library_path) const;
  bool loadControllerLibrary(LoadedControllerLibrary& out, std::string& err);
  static void unloadControllerLibrary(LoadedControllerLibrary& lib);
};

}  // namespace compliant_controllers
