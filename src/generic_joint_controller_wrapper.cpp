#include <compliant_controllers/generic_joint_controller_wrapper.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

namespace compliant_controllers {

namespace {
Eigen::VectorXd vector_or_constant(const std::vector<double>& values, int size, double fallback) {
  Eigen::VectorXd out(size);
  if (values.size() == static_cast<size_t>(size)) {
    out = Eigen::Map<const Eigen::VectorXd>(values.data(), size);
  } else if (values.size() == 1) {
    out.setConstant(values.front());
  } else {
    out.setConstant(fallback);
  }
  return out;
}
}  // namespace

CallbackReturn GenericJointControllerWrapper::on_init() {
  joint_command_sub_ = get_node()->create_subscription<compliant_controllers_msgs::msg::JointCommand>(
    "joint_command", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&GenericJointControllerWrapper::joint_command_callback, this, std::placeholders::_1));

  logger_ = get_node()->get_logger();
  configureLogging("GenericJointControllerWrapper");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericJointControllerWrapper::on_configure(const rclcpp_lifecycle::State&) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger_, "'joints' parameter must not be empty. Define the controlled joint names in the controller YAML.");
    return CallbackReturn::ERROR;
  }
  num_joints_ = static_cast<int>(joint_names_.size());

  std::vector<double> initial_stiffness;
  std::vector<double> initial_damping;
  get_node()->get_parameter("joint_impedance.initial_stiffness", initial_stiffness);
  get_node()->get_parameter("joint_impedance.initial_damping", initial_damping);
  double stiffness_fallback{0.0};
  double damping_fallback{0.0};
  get_node()->get_parameter("joint_impedance.initial_stiffness", stiffness_fallback);
  get_node()->get_parameter("joint_impedance.initial_damping", damping_fallback);
  init_stiffness_ = vector_or_constant(initial_stiffness, num_joints_, stiffness_fallback);
  init_damping_ = vector_or_constant(initial_damping, num_joints_, damping_fallback);

  add_friction_compensation_ = false;
  get_node()->get_parameter("friction_compensation_enabled", add_friction_compensation_);
  friction_compensation_.configure(get_node(), num_joints_, true);

  impl_library_ = get_node()->get_parameter("impl_library").as_string();

  std::string ee_frame_hint;
  get_node()->get_parameter("ee_frame", ee_frame_hint);
  if (!robot_description_loader_.load(get_node())) {
    return CallbackReturn::ERROR;
  }
  if (!robot_model_.init(robot_description_loader_.urdfXml(), ee_frame_hint, joint_names_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RobotModel with provided URDF");
    return CallbackReturn::ERROR;
  }
  if (!joint_limit_repulsion_.configure(
        get_node(), robot_description_loader_.urdfXml(), joint_names_)) {
    return CallbackReturn::ERROR;
  }

  std::string load_err;
  if (!instantiateImplementation(load_err)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to instantiate joint implementation: %s", load_err.c_str());
    return CallbackReturn::ERROR;
  }
  logConfigurationSummary();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericJointControllerWrapper::on_activate(const rclcpp_lifecycle::State&) {
  state_buffer_ = control::ControllerState(num_joints_);
  readControllerState(state_buffer_);
  robot_model_.update(state_buffer_.q);
  active_since_ = get_node()->get_clock()->now();
  first_update_logged_ = false;

  control::ControlCommand init_cmd(static_cast<size_t>(num_joints_));
  init_cmd.joint_position = state_buffer_.q;
  init_cmd.joint_velocity.setZero();
  init_cmd.joint_stiffness = init_stiffness_;
  init_cmd.joint_damping = init_damping_;
  init_cmd.joint_torque_ff.setZero();
  rt_joint_cmd_buffer_.writeFromNonRT(init_cmd);

  tau_out_.setZero(num_joints_);
  constexpr double controller_period = 0.001;
  const bool init_step_ok = impl_ && impl_->step(init_cmd, state_buffer_, tau_out_, controller_period);
  if (!init_step_ok) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Joint impl '%s' failed during activation step(); refusing activation.",
                 name_fn_ ? name_fn_() : "<unknown>");
    writeZeroTorques();
    return CallbackReturn::ERROR;
  }
  writeZeroTorques();

  logActivationSummary();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericJointControllerWrapper::on_deactivate(const rclcpp_lifecycle::State&) {
  writeZeroTorques();
  return CallbackReturn::SUCCESS;
}

void GenericJointControllerWrapper::joint_command_callback(
  const compliant_controllers_msgs::msg::JointCommand::SharedPtr msg) {
  const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  if (!has_stamp) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Dropping JointCommand with invalid zero timestamp.");
    return;
  }
  const rclcpp::Time msg_stamp(msg->header.stamp, RCL_ROS_TIME);
  const auto now = get_node()->get_clock()->now();
  if (msg_stamp < active_since_ || msg_stamp < now - rclcpp::Duration::from_seconds(0.5)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Dropping stale JointCommand.");
    return;
  }
  const auto expected = static_cast<size_t>(num_joints_);
  if (msg->position.size() != expected || msg->velocity.size() != expected ||
      msg->stiffness.size() != expected || msg->damping.size() != expected) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Dropping JointCommand with invalid dimensions.");
    return;
  }

  control::ControlCommand cmd(static_cast<size_t>(num_joints_));
  cmd.joint_position = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), num_joints_);
  cmd.joint_velocity = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), num_joints_);
  cmd.joint_stiffness = Eigen::Map<const Eigen::VectorXd>(msg->stiffness.data(), num_joints_);
  cmd.joint_damping = Eigen::Map<const Eigen::VectorXd>(msg->damping.data(), num_joints_);
  if (msg->torques_ff.size() == expected) {
    cmd.joint_torque_ff = Eigen::Map<const Eigen::VectorXd>(msg->torques_ff.data(), num_joints_);
  } else {
    cmd.joint_torque_ff.setZero();
  }
  rt_joint_cmd_buffer_.writeFromNonRT(cmd);
}

controller_interface::return_type GenericJointControllerWrapper::update(
  const rclcpp::Time&, const rclcpp::Duration& period) {
  readControllerState(state_buffer_);
  if (!robot_model_.update(state_buffer_.q)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "RobotModel update failed; writing zero torque.");
    writeZeroTorques();
    return controller_interface::return_type::OK;
  }
  const auto* latest_cmd = rt_joint_cmd_buffer_.readFromRT();
  if (!(impl_ && latest_cmd && impl_->step(*latest_cmd, state_buffer_, tau_out_, period.seconds()))) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Joint impl '%s' reported step() failure.",
                 name_fn_ ? name_fn_() : "<unknown>");
    writeZeroTorques();
    return controller_interface::return_type::ERROR;
  }
  if (add_friction_compensation_) {
    friction_compensation_.add(state_buffer_.q, state_buffer_.dq, period.seconds(), tau_out_);
  }
  if (!joint_limit_repulsion_.addTorque(state_buffer_.q, state_buffer_.dq, tau_out_)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Joint limit repulsion produced invalid torque; writing zero torque.");
    writeZeroTorques();
    return controller_interface::return_type::OK;
  }
  sanitizeTorqueOutput();
  writeTorqueOutput();
  logFirstUpdateSummary();
  return controller_interface::return_type::OK;
}

void GenericJointControllerWrapper::logConfigurationSummary() {
  RCLCPP_INFO(get_node()->get_logger(),
              "Joint wrapper configured: arm_id='%s', joints=%d, urdf='%s', impl='%s'",
              arm_id_.c_str(), num_joints_,
              robot_description_loader_.sourceDescription().c_str(),
              name_fn_ ? name_fn_() : "<unknown>");
}

void GenericJointControllerWrapper::logActivationSummary() {
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Joint wrapper activated: impl='" << (name_fn_ ? name_fn_() : "<unknown>")
                     << "', q=[" << state_buffer_.q.transpose()
                     << "], stiffness=[" << init_stiffness_.transpose()
                     << "], damping=[" << init_damping_.transpose() << "]");
}

void GenericJointControllerWrapper::logFirstUpdateSummary() {
  if (first_update_logged_) {
    return;
  }
  first_update_logged_ = true;
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "First joint update completed: impl='" << (name_fn_ ? name_fn_() : "<unknown>")
                     << "', tau_out=[" << tau_out_.transpose()
                     << "], q=[" << state_buffer_.q.transpose()
                     << "], dq=[" << state_buffer_.dq.transpose() << "]");
}

}  // namespace compliant_controllers

PLUGINLIB_EXPORT_CLASS(compliant_controllers::GenericJointControllerWrapper,
                       controller_interface::ControllerInterface)
