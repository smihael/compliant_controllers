#include <compliant_controllers/cartesian_impedance_controller.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace compliant_controllers {

controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<double>("k_p", 200.0);
    auto_declare<double>("k_d", 10.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  double k_p = get_node()->get_parameter("k_p").as_double();
  double k_d = get_node()->get_parameter("k_d").as_double();
  k_p_.fill(k_p);
  k_d_.fill(k_d);
  joint_command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~command", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->position.size() < static_cast<size_t>(num_joints_)) { return; }
      Command cmd;
      for (int i = 0; i < num_joints_; ++i) { cmd.q[i] = msg->position[i]; }
      cmd.has_value = true;
      command_buffer_.writeFromNonRT(cmd);
    });
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  first_update_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & ci : command_interfaces_) { ci.set_value(0.0); }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  std::array<double,7> q{}; std::array<double,7> dq{};
  for (int i = 0; i < num_joints_; ++i) {
    q[i] = state_interfaces_[2*i].get_value();
    dq[i] = state_interfaces_[2*i + 1].get_value();
  }
  if (first_update_) {
    last_position_ = q;
    last_velocity_ = dq;
    Command initial; initial.q = q; initial.has_value = true; command_buffer_.writeFromNonRT(initial);
    first_update_ = false;
  }
  Command cmd = *command_buffer_.readFromRT();
  if (!cmd.has_value) { cmd.q = q; }
  for (int i = 0; i < num_joints_; ++i) {
    double e = cmd.q[i] - q[i];
    double de = -dq[i];
    double tau = k_p_[i]*e + k_d_[i]*de;
    command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

} // namespace compliant_controllers

PLUGINLIB_EXPORT_CLASS(compliant_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
