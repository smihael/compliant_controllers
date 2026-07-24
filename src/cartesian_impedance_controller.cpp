#include <compliant_controllers/cartesian_impedance_controller.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/string.hpp>

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
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/effort");
  }

  return config;
}

CallbackReturn CartesianImpedanceController::on_init() {
  // Cartesian command subscription (non RT -> pushes into realtime buffer)
  cartesian_command_sub_ = get_node()->create_subscription<robot_module_msgs::msg::CartesianCommand>(
    "cartesian_command", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::cartesian_command_callback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_DEBUG(get_node()->get_logger(), "[CartesianImpedanceController] on_configure called");

  // Parameters (assume declared externally, else declare here)
  if (!get_node()->has_parameter("arm_id")) {
    get_node()->declare_parameter<std::string>("arm_id", "fr3");
  }
  if (!get_node()->has_parameter("k_p")) {
    get_node()->declare_parameter<double>("k_p", 100.0);
  }
  if (!get_node()->has_parameter("k_d")) {
    get_node()->declare_parameter<double>("k_d", 10.0);
  }
  if (!get_node()->has_parameter("robot_description_node")) {
    get_node()->declare_parameter<std::string>("robot_description_node", robot_description_node_);
  }
  if (!get_node()->has_parameter("robot_description_param")) {
    get_node()->declare_parameter<std::string>("robot_description_param", robot_description_param_);
  }
  if (!get_node()->has_parameter("ee_frame")) {
    get_node()->declare_parameter<std::string>("ee_frame", ""); // optional hint
  }

  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  double k_p = get_node()->get_parameter("k_p").as_double();
  double k_d = get_node()->get_parameter("k_d").as_double();
  k_p_.fill(k_p);
  k_d_.fill(k_d);

  // Update configured remote parameter identifiers
  robot_description_node_ = get_node()->get_parameter("robot_description_node").as_string();
  robot_description_param_ = get_node()->get_parameter("robot_description_param").as_string();

  // Attempt synchronous fetch of robot_description via AsyncParametersClient (bounded wait)
  std::string ee_frame_hint = get_node()->get_parameter("ee_frame").as_string();
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), robot_description_node_);
  if (!parameters_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameters service for node '%s' not available within timeout.", robot_description_node_.c_str());
    return CallbackReturn::ERROR;
  }
  try {
    auto future = parameters_client->get_parameters({robot_description_param_});
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(get_node()->get_logger(), "Timed out waiting for parameter '%s/%s'.", robot_description_node_.c_str(), robot_description_param_.c_str());
      return CallbackReturn::ERROR;
    }
    auto results = future.get();
    if (results.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' not returned.", robot_description_param_.c_str());
      return CallbackReturn::ERROR;
    }
    if (results.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' has wrong type.", robot_description_param_.c_str());
      return CallbackReturn::ERROR;
    }
    urdf_xml_ = results.front().as_string();
    if (urdf_xml_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Received empty URDF string from '%s/%s'.", robot_description_node_.c_str(), robot_description_param_.c_str());
      return CallbackReturn::ERROR;
    }
    urdf_received_.store(true);
  if (!robot_model_.init(urdf_xml_, ee_frame_hint)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RobotModel");
      return CallbackReturn::ERROR;
    }
  // allocate jacobian buffer with size equal to model dofs (fallback to num_joints_)
  int dofs = robot_model_.dofs() > 0 ? robot_model_.dofs() : num_joints_;
  J_.resize(6, dofs);
  RCLCPP_INFO(get_node()->get_logger(), "RobotModel DOFs: %d (controller joints: %d) EE frame: %s", dofs, num_joints_, robot_model_.endEffectorFrame().c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception fetching robot description: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
void CartesianImpedanceController::cartesian_command_callback(const robot_module_msgs::msg::CartesianCommand::SharedPtr msg) {
  CartesianCommandRT cmd;
  // Pose/velocity/acceleration/wrench
  cmd.position = Eigen::Vector3d(msg->pose_des.position.x, msg->pose_des.position.y, msg->pose_des.position.z);
  cmd.orientation = Eigen::Quaterniond(msg->pose_des.orientation.w, msg->pose_des.orientation.x,
                                       msg->pose_des.orientation.y, msg->pose_des.orientation.z).normalized();
  cmd.velocity.setZero();
  cmd.velocity.head(3) << msg->velocity_des.linear.x, msg->velocity_des.linear.y, msg->velocity_des.linear.z;
  cmd.velocity.tail(3) << msg->velocity_des.angular.x, msg->velocity_des.angular.y, msg->velocity_des.angular.z;
  cmd.wrench.setZero();
  cmd.wrench.head(3) << msg->wrench_ff.force.x, msg->wrench_ff.force.y, msg->wrench_ff.force.z;
  cmd.wrench.tail(3) << msg->wrench_ff.torque.x, msg->wrench_ff.torque.y, msg->wrench_ff.torque.z;

  // Impedance: two 3x3 row-major blocks for position(0:8) and orientation(9:17)
  cmd.stiffness.setZero();
  cmd.damping.setZero();
  if (msg->stiffness_pos.size() == 9 && msg->stiffness_ori.size() == 9) {
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Kp(msg->stiffness_pos.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Ko(msg->stiffness_ori.data());
    cmd.stiffness.block<3,3>(0,0) = Kp;
    cmd.stiffness.block<3,3>(3,3) = Ko;
  }
  if (msg->damping_pos.size() == 9 && msg->damping_ori.size() == 9) {
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Dp(msg->damping_pos.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Do(msg->damping_ori.data());
    cmd.damping.block<3,3>(0,0) = Dp;
    cmd.damping.block<3,3>(3,3) = Do;
  }

  // Nullspace & torque feedforward
  cmd.has_nullspace = false;
  const auto n_qd = msg->nullspace_position_des.size();
  const auto n_k = msg->nullspace_stiffness.size();
  if (n_qd > 0 && n_k == n_qd) {
    cmd.q_ns_des = Eigen::Map<const Eigen::VectorXd>(msg->nullspace_position_des.data(), static_cast<long>(n_qd));
    cmd.k_ns = Eigen::Map<const Eigen::VectorXd>(msg->nullspace_stiffness.data(), static_cast<long>(n_k));
    if (msg->nullspace_damping.size() == n_qd) {
      cmd.d_ns = Eigen::Map<const Eigen::VectorXd>(msg->nullspace_damping.data(), static_cast<long>(n_qd));
    } else {
      cmd.d_ns = Eigen::VectorXd::Zero(static_cast<long>(n_qd));
    }
    cmd.has_nullspace = true;
  }
  cmd.has_tau_ff = false;
  if (!msg->torques_ff.empty()) {
    cmd.tau_ff = Eigen::Map<const Eigen::VectorXd>(msg->torques_ff.data(), static_cast<long>(msg->torques_ff.size()));
    cmd.has_tau_ff = true;
  }

  cmd.has_value = true;
  rt_cartesian_cmd_buffer_.writeFromNonRT(cmd);
}

Eigen::Vector3d CartesianImpedanceController::q_log(const Eigen::Quaterniond &q) const {
  Eigen::Quaterniond qn = q.normalized();
  double w = std::clamp(qn.w(), -1.0, 1.0);
  double angle = 2.0 * std::acos(w);
  double s = std::sqrt(1 - w * w);
  if (s < 1e-8 || angle < 1e-8) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d axis(qn.x() / s, qn.y() / s, qn.z() / s);
  return angle * axis / 2.0;
}

void CartesianImpedanceController::pseudoInverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_pinv, double tolerance) const {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tol = tolerance * std::max(M.cols(), M.rows()) * svd.singularValues().array().abs()(0);
  Eigen::VectorXd singular_inv = svd.singularValues();
  for (int i = 0; i < singular_inv.size(); ++i) {
    singular_inv(i) = (singular_inv(i) > tol) ? 1.0 / singular_inv(i) : 0.0;
  }
  M_pinv = svd.matrixV() * singular_inv.asDiagonal() * svd.matrixU().transpose();
}

CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_DEBUG(get_node()->get_logger(), "[CartesianImpedanceController] on_activate called");
  first_update_ = true;

  position_interfaces_.clear();
  velocity_interfaces_.clear();
  effort_interfaces_.clear();

  q.setZero();
  dq.setZero();
  tau.setZero();

  for (int i = 0; i < num_joints_; ++i) {
    position_interfaces_.push_back(&state_interfaces_.at(3 * i));
    velocity_interfaces_.push_back(&state_interfaces_.at(3 * i + 1));
    effort_interfaces_.push_back(&state_interfaces_.at(3 * i + 2));

    q(i) = position_interfaces_[i]->get_value();
    dq(i) = velocity_interfaces_[i]->get_value();
    tau(i) = effort_interfaces_[i]->get_value();
  }

  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::ostringstream oss;
  oss << q.transpose().format(fmt);

  RCLCPP_INFO(get_node()->get_logger(),
              "[CartesianImpedanceController] Initial joint positions: %s",
              oss.str().c_str());

  q_d_nullspace_ = q;
  nullspace_stiffness_.setConstant(10.0);

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  for (int i = 0; i < 3; ++i) {
    cartesian_stiffness_(i,i) = 300.0; 
    cartesian_damping_(i,i) = 2.0 * std::sqrt(cartesian_stiffness_(i,i));
  }
  for (int i = 3; i < 6; ++i) {
    cartesian_stiffness_(i,i) = 20.0; 
    cartesian_damping_(i,i) = 2.0 * std::sqrt(cartesian_stiffness_(i,i));
  }

  if (urdf_received_) {
    Eigen::Vector3d pos; Eigen::Quaterniond ori;
    if (robot_model_.update(q) && robot_model_.getPose(pos, ori)) {
      position_d_ = pos;
      orientation_d_ = ori;
      RCLCPP_INFO(get_node()->get_logger(), "Desired pose initialized from current EE pose (RobotModel).");
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to initialize desired pose from RobotModel.");
    }
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "[CartesianImpedanceController] on_activate finished");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & ci : command_interfaces_) { ci.set_value(0.0); }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  for (int i = 0; i < num_joints_; ++i) {
    q(i) = position_interfaces_[i]->get_value();
    dq(i) = velocity_interfaces_[i]->get_value();
    tau(i) = effort_interfaces_[i]->get_value();
  }

  if (auto * latest = rt_cartesian_cmd_buffer_.readFromRT(); latest && latest->has_value) {
    position_d_ = latest->position;
    orientation_d_ = latest->orientation;
    velocity_d_ = latest->velocity;
    wrench_d_ = latest->wrench;
    if (latest->stiffness.norm() > 0.0) { cartesian_stiffness_ = latest->stiffness; }
    if (latest->damping.norm() > 0.0) { cartesian_damping_ = latest->damping; }
    // Nullspace updates (size-checked)
    if (latest->has_nullspace) {
      if (latest->q_ns_des.size() == num_joints_) {
        q_d_nullspace_ = latest->q_ns_des;
      } else if (latest->q_ns_des.size() > 0) {
        // partial copy if sizes mismatch; clamp to min size
        int n = std::min<int>(num_joints_, latest->q_ns_des.size());
        q_d_nullspace_.head(n) = latest->q_ns_des.head(n);
      }
      if (latest->k_ns.size() == num_joints_) {
        nullspace_stiffness_ = latest->k_ns;
      } else if (latest->k_ns.size() > 0) {
        int n = std::min<int>(num_joints_, latest->k_ns.size());
        nullspace_stiffness_.head(n) = latest->k_ns.head(n);
      }
    }
  }

  if (urdf_received_) {
    try {
      if(!robot_model_.update(q)) throw std::runtime_error("RobotModel update failed");
      if(J_.cols() != num_joints_) J_.resize(6, num_joints_);
      if(!robot_model_.getJacobian(J_)) throw std::runtime_error("Jacobian fetch failed");
      Eigen::Vector3d position; Eigen::Quaterniond orientation;
      if(!robot_model_.getPose(position, orientation)) throw std::runtime_error("Pose fetch failed");
      Eigen::Matrix<double,6,1> error; error.setZero();
      error.head(3) = position - position_d_;
      Eigen::Quaterniond orientation_curr = orientation;
      Eigen::Quaterniond orientation_des = orientation_d_;
      if (orientation_des.coeffs().dot(orientation_curr.coeffs()) < 0.0) {
        orientation_curr.coeffs() *= -1.0;
      }
      Eigen::Quaterniond error_quaternion(orientation_curr.inverse() * orientation_des);
      Eigen::Vector3d lq = 2.0 * q_log(error_quaternion);
      error.tail(3) << lq.x(), lq.y(), lq.z();
      // Using negative rotation like original (placement.rotation() approximated by orientation matrix)
      Eigen::Matrix3d R = orientation.toRotationMatrix();
      error.tail(3) = -R * error.tail(3);

      Eigen::Matrix<double,6,1> velocity_error = J_ * dq - velocity_d_;

      Eigen::MatrixXd jacobian_transpose_pinv;
      pseudoInverse(J_.transpose(), jacobian_transpose_pinv);

      Eigen::Matrix<double,7,1> tau_task = J_.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * velocity_error);
      Eigen::Matrix<double,7,1> tau_ft_added = J_.transpose() * wrench_d_;
      Eigen::Matrix<double,7,1> tau_nullspace = (Eigen::Matrix<double,7,7>::Identity() - J_.transpose() * jacobian_transpose_pinv)
        * (nullspace_stiffness_.array() * (q_d_nullspace_ - q).array()).matrix();
      if (tau_nullspace.norm() > TAU_NULLSPACE_MAX_) {
        tau_nullspace = tau_nullspace * TAU_NULLSPACE_MAX_ / tau_nullspace.norm();
      }

      Eigen::Matrix<double,7,1> coriolis = Eigen::Matrix<double,7,1>::Zero();
      Eigen::Matrix<double,7,1> tau_d = tau_task + tau_nullspace + coriolis + tau_ft_added;
      // Add joint torque feedforward if provided
      if (auto * latest = rt_cartesian_cmd_buffer_.readFromRT(); latest && latest->has_tau_ff) {
        if (latest->tau_ff.size() == num_joints_) {
          tau_d += latest->tau_ff;
        } else if (latest->tau_ff.size() > 0) {
          int n = std::min<int>(num_joints_, latest->tau_ff.size());
          tau_d.head(n) += latest->tau_ff.head(n);
        }
      }

      for (int i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, "Control update failed: %s", e.what());
      for (int i = 0; i < num_joints_; ++i) { command_interfaces_[i].set_value(0.0); }
    }
  } else {
    for (int i = 0; i < num_joints_; ++i) { command_interfaces_[i].set_value(0.0); }
  }
  return controller_interface::return_type::OK;
}

} // namespace compliant_controllers

PLUGINLIB_EXPORT_CLASS(compliant_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
