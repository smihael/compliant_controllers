#include <compliant_controllers/generic_cartesian_controller_wrapper.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>

namespace compliant_controllers {

namespace {

}  // namespace

CallbackReturn GenericCartesianControllerWrapper::on_init() {

  logger_ = get_node()->get_logger();
  configureLogging("GenericCartesianControllerWrapper");
  auto_declare<double>("max_position_command_step_m", max_position_command_step_m_);
  auto_declare<double>("max_orientation_command_step_rad", max_orientation_command_step_rad_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCartesianControllerWrapper::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

  // Cartesian command subscription (non RT -> pushes into realtime buffer)
  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1));

  cartesian_command_sub_ = get_node()->create_subscription<compliant_controllers_msgs::msg::CartesianCommand>(
    "cartesian_command", command_qos,
    std::bind( &GenericCartesianControllerWrapper::cartesian_command_callback, this, std::placeholders::_1));

  position_stiffness_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "stiffness_pos", command_qos,
    std::bind(&GenericCartesianControllerWrapper::position_stiffness_callback,
              this, std::placeholders::_1));
  orientation_stiffness_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "stiffness_ori", command_qos,
    std::bind(&GenericCartesianControllerWrapper::orientation_stiffness_callback,
              this, std::placeholders::_1));

  // Check if explicit joint names are provided
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger_, "'joints' parameter must not be empty. Define the controlled joint names in the controller YAML.");
    return CallbackReturn::ERROR;
  }
  num_joints_ = static_cast<int>(joint_names_.size());
  RCLCPP_INFO(logger_, "Using %zu configured joints", joint_names_.size());
  dither_jacobian_.resize(6, num_joints_);
  dither_torque_.setZero(num_joints_);

  get_node()->get_parameter("arm_id", arm_id_);
  get_node()->get_parameter("init_k_pos", init_k_pos_);
  get_node()->get_parameter("init_k_ori", init_k_ori_);
  get_node()->get_parameter("gravity_compensation_enabled", robot_gravity_compensation_enabled_);
  get_node()->get_parameter("dithering_enabled", dithering_enabled_);
  get_node()->get_parameter("max_position_command_step_m", max_position_command_step_m_);
  get_node()->get_parameter("max_orientation_command_step_rad", max_orientation_command_step_rad_);

  if (!friction_compensation_.configure(get_node(), num_joints_, true)) {
    RCLCPP_ERROR(logger_, "Failed to configure friction compensation.");
    return CallbackReturn::ERROR;
  }

  diagnostic_logger_.configure(get_node(), num_joints_);

  impl_library_ = get_node()->get_parameter("impl_library").as_string();

  std::string ee_frame_hint = get_node()->get_parameter("ee_frame").as_string();
  if (!robot_description_loader_.load(get_node())) {
    return CallbackReturn::ERROR;
  }

  // Initialize robot model owned by wrapper
  if(!robot_model_.init(robot_description_loader_.urdfXml(), ee_frame_hint, joint_names_)) {
    RCLCPP_ERROR(logger_, "Failed to initialize RobotModel with provided URDF");
    return CallbackReturn::ERROR;
  }
  if (!joint_limit_repulsion_.configure(
        get_node(), robot_description_loader_.urdfXml(), joint_names_)) {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Robot model initialized: modeled_frame='%s', controlled_dofs=%d",
              robot_model_.endEffectorFrame().c_str(),
              robot_model_.dofs());

  // Instantiate implementation only; defer initialize() until activation
  std::string load_err;
  if(!instantiateImplementation(load_err)) {
    RCLCPP_ERROR(logger_, "Failed to instantiate implementation: %s", load_err.c_str());
    return CallbackReturn::ERROR;
  }
  if (impl_) {
    impl_->setRobotModel(static_cast<void*>(&robot_model_));
  }
  RCLCPP_INFO(logger_, "Instantiated control implementation library (initialization deferred): %s", impl_library_.c_str());
  logConfigurationSummary();

  return CallbackReturn::SUCCESS;
}

void GenericCartesianControllerWrapper::logConfigurationSummary() {

  RCLCPP_INFO(logger_,
              "Wrapper configured: arm_id='%s', joints=%d, ee_frame='%s', urdf='%s', impl='%s'",
              arm_id_.c_str(), num_joints_,
              robot_model_.endEffectorFrame().c_str(),
              robot_description_loader_.sourceDescription().c_str(),
              name_fn_ ? name_fn_() : "<unknown>");
  RCLCPP_INFO(logger_,
              "Compensation configured: gravity=%s, friction=%s (scale=%.3f), dithering=%s",
              robot_gravity_compensation_enabled_ ? "enabled" : "disabled",
              friction_compensation_.enabled() ? friction_compensation_.modelName().c_str() : "disabled",
              friction_compensation_.scale(),
              dithering_enabled_ ? "enabled" : "disabled");
  RCLCPP_INFO(logger_,
              "Error-biased dither: %s, world-y rotation, frequency=%.1fHz, amplitude=%.3fNm, alpha=%.1f, beta=%.2f",
              dithering_enabled_ ? "enabled" : "disabled",
              CartesianErrorBiasedDither::kFrequencyHz,
              CartesianErrorBiasedDither::kBaseAmplitudeNm,
              CartesianErrorBiasedDither::kAlpha,
              CartesianErrorBiasedDither::kBeta);
  if (diagnostic_logger_.enabled()) {
    RCLCPP_INFO(logger_,
                "Diagnostic logging: enabled file='%s', duration=%.3fs, log_filter_tag=%d",
                diagnostic_logger_.outputPath().c_str(),
                diagnostic_logger_.duration(),
                diagnostic_logger_.logFilterTag());
  }
}

void GenericCartesianControllerWrapper::logActivationSummary() {
  RCLCPP_INFO(logger_,
              "Wrapper activated: impl='%s', command_interfaces=%zu, state_interfaces=%zu, initial_step=ok",
              name_fn_ ? name_fn_() : "<unknown>",
              command_interfaces_.size(),
              state_interfaces_.size());
  RCLCPP_INFO_STREAM(logger_,
                     "Initial state: q=[" << state_buffer_.q.transpose()
                     << "], tau_measured=[" << state_buffer_.tau.transpose()
                     << "], ee_position=[" << state_buffer_.position.transpose()
                     << "]");
  RCLCPP_DEBUG_STREAM(logger_,
                      "Initial command: stiffness_diag=["
                      << init_k_pos_ << ", " << init_k_pos_ << ", " << init_k_pos_
                      << ", " << init_k_ori_ << ", " << init_k_ori_ << ", " << init_k_ori_
                      << "], q_ns_des=[" << state_buffer_.q.transpose() << "]");
}

CallbackReturn GenericCartesianControllerWrapper::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  state_buffer_ = control::ControllerState(num_joints_);
  readControllerState(state_buffer_);
  robot_model_.update(state_buffer_.q);
  const bool pose_ok = robot_model_.getPose(state_buffer_.position, state_buffer_.orientation);
  if (!pose_ok) {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                         "RobotModel pose update failed; keeping previous EE state.");
  }
  active_since_ = get_node()->get_clock()->now();
  error_biased_dither_.reset();
  diagnostic_logger_.start();

  control::ControlCommand init_cmd(static_cast<size_t>(num_joints_));
  init_cmd.position = state_buffer_.position;
  init_cmd.orientation = state_buffer_.orientation;
  init_cmd.velocity.setZero();
  init_cmd.wrench.setZero();
  init_cmd.stiffness.setZero();
  init_cmd.damping.setZero();
  init_cmd.stiffness.topLeftCorner(3, 3) = init_k_pos_ * Eigen::Matrix3d::Identity();
  init_cmd.stiffness.bottomRightCorner(3, 3) = init_k_ori_ * Eigen::Matrix3d::Identity();
  init_cmd.damping.topLeftCorner(3, 3) = 2.0 * sqrt(init_k_pos_) * Eigen::Matrix3d::Identity();
  init_cmd.damping.bottomRightCorner(3, 3) = 2.0 * sqrt(init_k_ori_) * Eigen::Matrix3d::Identity();
  init_cmd.q_ns_des = state_buffer_.q;
  init_cmd.k_ns = 5.0 * Eigen::VectorXd::Ones(num_joints_);
  init_cmd.d_ns = 2.0 * std::sqrt(5.0) * Eigen::VectorXd::Ones(num_joints_);
  rt_cartesian_cmd_buffer_.writeFromNonRT(init_cmd);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_ = init_cmd;
    have_last_command_ = true;
  }

  tau_out_.setZero(num_joints_);
  constexpr double controller_period = 0.001;
  const bool init_step_ok = impl_ && impl_->step(init_cmd, state_buffer_, tau_out_, controller_period);
  if (!init_step_ok) {
    RCLCPP_ERROR(logger_,
                 "Controller impl '%s' failed during activation step(); refusing activation.",
                 name_fn_ ? name_fn_() : "<unknown>");
    writeZeroTorques();
    diagnostic_logger_.stop();
    return CallbackReturn::ERROR;
  }

  logActivationSummary();
  RCLCPP_DEBUG_STREAM(logger_, "Initial q: " << state_buffer_.q.transpose() << " | tau: " << state_buffer_.tau.transpose() << " | ee: [" << state_buffer_.position.transpose() << "]");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCartesianControllerWrapper::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  diagnostic_logger_.stop();
  for (auto & ci : command_interfaces_) { ci.set_value(0.0); }
  return CallbackReturn::SUCCESS;
}

void GenericCartesianControllerWrapper::cartesian_command_callback(const compliant_controllers_msgs::msg::CartesianCommand::SharedPtr msg) {

  const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  if (!has_stamp) {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                         "Dropping CartesianCommand with invalid (zero) header timestamp.");
    return;
  }

  const rclcpp::Time msg_stamp(msg->header.stamp, RCL_ROS_TIME);
  const rclcpp::Time now = get_node()->get_clock()->now();
  if (msg_stamp < active_since_) {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                         "Dropping CartesianCommand stamped before controller activation (%.3fs before activation).",
                         (active_since_ - msg_stamp).seconds());
    return;
  }
  if (msg_stamp < now - rclcpp::Duration::from_seconds(0.5)) {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                         "Dropping stale CartesianCommand (%.3fs old).",
                         (now - msg_stamp).seconds());
    return;
  }

  const auto expected_frame = robot_model_.endEffectorFrame();
  if (!expected_frame.empty()) {
    auto normalize_frame = [](std::string frame) {
      while (!frame.empty() && frame.front() == '/') {
        frame.erase(frame.begin());
      }
      return frame;
    };
    const auto message_frame = normalize_frame(msg->header.frame_id);
    const auto expected = normalize_frame(expected_frame);
    if (message_frame.empty()) {
      RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                           "CartesianCommand header.frame_id is empty; expected controlled frame '%s'.",
                           expected_frame.c_str());
    } else if (message_frame != expected) {
      RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                           "Dropping CartesianCommand for frame '%s'; controller frame is '%s'.",
                           msg->header.frame_id.c_str(), expected_frame.c_str());
      return;
    }
  }

  control::ControlCommand cmd(static_cast<size_t>(num_joints_));

  cmd.position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

  cmd.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                       msg->pose.orientation.y, msg->pose.orientation.z).normalized();

  const double position_step = (cmd.position - state_buffer_.position).norm();
  const double orientation_dot = std::clamp(
      std::abs(cmd.orientation.dot(state_buffer_.orientation.normalized())), 0.0, 1.0);
  const double orientation_step = 2.0 * std::acos(orientation_dot);
  if (!std::isfinite(position_step) || position_step > max_position_command_step_m_) {
    RCLCPP_WARN_THROTTLE(
        logger_, *get_node()->get_clock(), 1000,
        "CartesianCommand translation jump is %.3f mm (limit %.3f mm); "
        "using the current end-effector position.",
        1000.0 * position_step, 1000.0 * max_position_command_step_m_);
    cmd.position = state_buffer_.position;
  }
  if (!std::isfinite(orientation_step) ||
      orientation_step > max_orientation_command_step_rad_) {
    RCLCPP_WARN_THROTTLE(
        logger_, *get_node()->get_clock(), 1000,
        "CartesianCommand orientation jump is %.3f deg (limit %.3f deg); "
        "using the current end-effector orientation.",
        180.0 * orientation_step / std::numbers::pi,
        180.0 * max_orientation_command_step_rad_ / std::numbers::pi);
    cmd.orientation = state_buffer_.orientation;
  }

  if (cmd.orientation.coeffs().dot(state_buffer_.orientation.coeffs()) < 0.0) {
    cmd.orientation.coeffs() *= -1.0;
  }

  cmd.velocity.head(3) << msg->velocity.linear.x, msg->velocity.linear.y, msg->velocity.linear.z;
  cmd.velocity.tail(3) << msg->velocity.angular.x, msg->velocity.angular.y, msg->velocity.angular.z;
  cmd.wrench.head(3) << msg->wrench_ff.force.x, msg->wrench_ff.force.y, msg->wrench_ff.force.z;
  cmd.wrench.tail(3) << msg->wrench_ff.torque.x, msg->wrench_ff.torque.y, msg->wrench_ff.torque.z;

  cmd.stiffness.setZero();
  cmd.damping.setZero();
  // Cartesian Impedance: two 3x3 row-major blocks for position(0:8) and orientation(9:17)
  if (msg->stiffness_pos.size() == 9 && msg->stiffness_ori.size() == 9) {
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Kp(msg->stiffness_pos.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Ko(msg->stiffness_ori.data());
    cmd.stiffness.topLeftCorner(3, 3) = Kp;
    cmd.stiffness.bottomRightCorner(3, 3) = Ko;
  }
  if (msg->damping_pos.size() == 9 && msg->damping_ori.size() == 9) {
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Dp(msg->damping_pos.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Do(msg->damping_ori.data());
    cmd.damping.topLeftCorner(3, 3) = Dp;
    cmd.damping.bottomRightCorner(3, 3) = Do;
  }

  // Nullspace & torque feedforward
  const auto n_qd = msg->nullspace_position.size();
  const auto n_k = msg->nullspace_stiffness.size();
  if (n_qd > 0 && n_k == n_qd) {
    cmd.q_ns_des = Eigen::Map<const Eigen::VectorXd>(msg->nullspace_position.data(), static_cast<long>(n_qd));
    cmd.k_ns = Eigen::Map<const Eigen::VectorXd>(msg->nullspace_stiffness.data(), static_cast<long>(n_k));
    if (msg->nullspace_damping.size() == n_qd) {
      cmd.d_ns = Eigen::Map<const Eigen::VectorXd>(msg->nullspace_damping.data(), static_cast<long>(n_qd));
    } else {
      cmd.d_ns = Eigen::VectorXd::Zero(static_cast<long>(n_qd));
    }
  }
  if (!msg->torques_ff.empty()) {
    cmd.tau_ff = Eigen::Map<const Eigen::VectorXd>(msg->torques_ff.data(), static_cast<long>(msg->torques_ff.size()));
  }

  // print all cmd fields
  std::ostringstream oss;
  oss << "Received CartesianCommand:\n";
  oss << "  position: [" << cmd.position.x() << ", " << cmd.position.y() << ", " << cmd.position.z() << "]\n";
  oss << "  orientation: [" << cmd.orientation.w() << ", " << cmd.orientation.x() << ", " << cmd.orientation.y() << ", " << cmd.orientation.z() << "]\n";
  oss << "  velocity: " << cmd.velocity.transpose() << "\n";
  oss << "  wrench: " << cmd.wrench.transpose() << "\n";
  oss << "  stiffness:\n" << cmd.stiffness << "\n";
  oss << "  damping:\n" << cmd.damping << "\n";
  oss << "  nullspace desired position: " << cmd.q_ns_des.transpose() << "\n";
  oss << "  nullspace stiffness: " << cmd.k_ns.transpose() << "\n";
  oss << "  nullspace damping: " << cmd.d_ns.transpose() << "\n";
  oss << "  torque feedforward: " << cmd.tau_ff.transpose() << "\n";  
  RCLCPP_DEBUG(logger_, "%s", oss.str().c_str());

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_ = cmd;
    have_last_command_ = true;
    rt_cartesian_cmd_buffer_.writeFromNonRT(last_command_);
  }

}

void GenericCartesianControllerWrapper::position_stiffness_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != 9) {
    RCLCPP_ERROR(logger_, "Position stiffness must contain exactly 9 row-major values; got %zu.",
                 msg->data.size());
    return;
  }

  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> input(msg->data.data());
  if (!input.array().isFinite().all() || !input.isApprox(input.transpose(), 1e-9)) {
    RCLCPP_ERROR(logger_, "Position stiffness must be finite and symmetric.");
    return;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(input);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-9) {
    RCLCPP_ERROR(logger_, "Position stiffness must be positive semidefinite.");
    return;
  }
  const Eigen::Vector3d damping_eigenvalues =
    2.0 * solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  const Eigen::Matrix3d damping =
    solver.eigenvectors() * damping_eigenvalues.asDiagonal() * solver.eigenvectors().transpose();

  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!have_last_command_) {
    RCLCPP_WARN(logger_, "Ignoring stiffness update before the controller reference is initialized.");
    return;
  }
  last_command_.stiffness.topLeftCorner<3, 3>() = input;
  last_command_.damping.topLeftCorner<3, 3>() = damping;
  rt_cartesian_cmd_buffer_.writeFromNonRT(last_command_);
  RCLCPP_INFO_STREAM(logger_, "Position stiffness updated without changing the Cartesian reference:\n"
                     << input);
}

void GenericCartesianControllerWrapper::orientation_stiffness_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != 9) {
    RCLCPP_ERROR(logger_, "Orientation stiffness must contain exactly 9 row-major values; got %zu.",
                 msg->data.size());
    return;
  }

  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> input(msg->data.data());
  if (!input.array().isFinite().all() || !input.isApprox(input.transpose(), 1e-9)) {
    RCLCPP_ERROR(logger_, "Orientation stiffness must be finite and symmetric.");
    return;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(input);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-9) {
    RCLCPP_ERROR(logger_, "Orientation stiffness must be positive semidefinite.");
    return;
  }
  const Eigen::Vector3d damping_eigenvalues =
    2.0 * solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  const Eigen::Matrix3d damping =
    solver.eigenvectors() * damping_eigenvalues.asDiagonal() * solver.eigenvectors().transpose();

  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!have_last_command_) {
    RCLCPP_WARN(logger_, "Ignoring stiffness update before the controller reference is initialized.");
    return;
  }
  last_command_.stiffness.bottomRightCorner<3, 3>() = input;
  last_command_.damping.bottomRightCorner<3, 3>() = damping;
  rt_cartesian_cmd_buffer_.writeFromNonRT(last_command_);
  RCLCPP_INFO_STREAM(logger_, "Orientation stiffness updated without changing the Cartesian reference:\n"
                     << input);
}

controller_interface::return_type GenericCartesianControllerWrapper::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {

  readControllerState(state_buffer_);   // updates joint positions, velocities, and torques from hardware interface
  const bool model_ok = robot_model_.update(state_buffer_.q); // passes current joint positions to the robot model
  if (!model_ok) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                          "RobotModel update failed; writing zero torque for this cycle.");
    writeZeroTorques();
    return controller_interface::return_type::OK;
  }

  const bool pose_ok = robot_model_.getPose(state_buffer_.position, state_buffer_.orientation);
  if (!pose_ok) {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                         "RobotModel pose update failed; keeping previous EE state.");
  }
  // read latest command from RT buffer and pass it to the implementation
  const control::ControlCommand* latest_cmd = rt_cartesian_cmd_buffer_.readFromRT();
  // call step function of the controller implementation, returning control output torques
  const bool step_ok = impl_ && impl_->step(*latest_cmd, state_buffer_, tau_out_, period.seconds());
  if (!step_ok) {
    RCLCPP_ERROR(logger_,
                 "Controller impl '%s' reported step() failure; writing zero torque and returning ERROR.",
                 name_fn_ ? name_fn_() : "<unknown>");
    writeZeroTorques();
    return controller_interface::return_type::ERROR;
  }

  Eigen::VectorXd gravity;
  bool gravity_ok = false;
  if (robot_gravity_compensation_enabled_ || diagnostic_logger_.enabled()) {
    gravity.setZero(num_joints_);
    gravity_ok = robot_model_.getGravity(gravity);
    if (robot_gravity_compensation_enabled_ && !gravity_ok) {
      RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                           "Gravity compensation skipped because the robot model could not provide gravity torques.");
    } else if (robot_gravity_compensation_enabled_ && !gravity.array().isFinite().all()) {
      RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                           "Gravity compensation skipped because the robot model returned non-finite torques.");
      gravity_ok = false;
    }
    if (robot_gravity_compensation_enabled_ && gravity_ok) {
      tau_out_ += gravity;
    }
  }
  addFrictionCompensation(period.seconds());
  addErrorBiasedDither(*latest_cmd, period.seconds());
  if (!joint_limit_repulsion_.addTorque(state_buffer_.q, state_buffer_.dq, tau_out_)) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                          "Joint limit repulsion produced invalid torque; writing zero torque.");
    writeZeroTorques();
    return controller_interface::return_type::OK;
  }
  sanitizeTorqueOutput();
  if (diagnostic_logger_.enabled()) {
    Eigen::VectorXd coriolis = Eigen::VectorXd::Zero(num_joints_);
    Eigen::MatrixXd inertia = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
    robot_model_.getCoriolis(state_buffer_.dq, coriolis);
    robot_model_.getMassMatrix(inertia);
    diagnostic_logger_.record(get_node()->get_clock()->now().seconds(),
                              state_buffer_.q,
                              state_buffer_.dq,
                              state_buffer_.tau,
                              tau_out_,
                              gravity,
                              coriolis,
                              inertia);
  }
  
  writeTorqueOutput();
  
  return controller_interface::return_type::OK;
}

void GenericCartesianControllerWrapper::addFrictionCompensation(double dt) {
  if (!friction_compensation_.add(state_buffer_.q, state_buffer_.dq, dt, tau_out_)) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                          "Friction compensation failed; disabling compensation.");
  }
}

void GenericCartesianControllerWrapper::addErrorBiasedDither(
    const control::ControlCommand& command, double dt) {
  if (!dithering_enabled_) {
    return;
  }

  if (!error_biased_dither_.compute(command, state_buffer_, dt, dither_wrench_)) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                          "Error-biased Cartesian dither computation failed; skipping this cycle.");
    return;
  }
  if (!robot_model_.getJacobian(dither_jacobian_) ||
      !dither_jacobian_.array().isFinite().all()) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                          "Error-biased Cartesian dither could not obtain a finite Jacobian.");
    return;
  }

  dither_torque_.noalias() = dither_jacobian_.transpose() * dither_wrench_;
  if (!dither_torque_.array().isFinite().all()) {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                          "Error-biased Cartesian dither produced non-finite joint torque.");
    return;
  }
  tau_out_ += dither_torque_;
}


} // namespace compliant_controllers

PLUGINLIB_EXPORT_CLASS(compliant_controllers::GenericCartesianControllerWrapper, controller_interface::ControllerInterface)
