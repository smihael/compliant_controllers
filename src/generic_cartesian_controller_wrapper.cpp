#include <compliant_controllers/generic_cartesian_controller_wrapper.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
// dlopen for external implementations
#include <dlfcn.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <std_msgs/msg/string.hpp>
#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>

namespace compliant_controllers {

controller_interface::InterfaceConfiguration GenericCartesianControllerWrapper::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration GenericCartesianControllerWrapper::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/effort");
  }

  return config;
}

void GenericCartesianControllerWrapper::readControllerState(control::ControllerState& out) {
  for (int i = 0; i < num_joints_; ++i) {
    out.q(i) = state_interfaces_.at(3 * i).get_value();
    out.dq(i) = state_interfaces_.at(3 * i + 1).get_value();
    out.tau(i) = state_interfaces_.at(3 * i + 2).get_value();
  }
}

CallbackReturn GenericCartesianControllerWrapper::on_init() {
  // Cartesian command subscription (non RT -> pushes into realtime buffer)
  cartesian_command_sub_ = get_node()->create_subscription<compliant_controllers_msgs::msg::CartesianCommand>(
    "cartesian_command", rclcpp::SystemDefaultsQoS(),
    std::bind(&GenericCartesianControllerWrapper::cartesian_command_callback, this, std::placeholders::_1));

  // Print in green using ANSI escape sequence
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "\033[32mUsing GenericCartesianControllerWrapper compiled at " << __DATE__ << ", " << __TIME__ << "\033[0m");

  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCartesianControllerWrapper::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  // The following parameters are expected to be declared externally (e.g., via launch or YAML):

  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  init_k_pos_ = get_node()->get_parameter("init_k_pos").as_double();
  init_k_ori_ = get_node()->get_parameter("init_k_ori").as_double();

  // Update configured remote parameter identifiers
  robot_description_node_ = get_node()->get_parameter("robot_description_node").as_string();
  robot_description_param_ = get_node()->get_parameter("robot_description_param").as_string();

  impl_library_ = get_node()->get_parameter("impl_library").as_string();
  if (!impl_library_.empty()) {
    // Resolve relative paths: if not absolute, try as relative to this package share dir and its lib dir
    if (!std::filesystem::path(impl_library_).is_absolute()) {
      try {
        std::string share = ament_index_cpp::get_package_share_directory("compliant_controllers");
        std::filesystem::path candidate1 = std::filesystem::path(share) / impl_library_;
        std::filesystem::path candidate2 = std::filesystem::path(share) / "../lib" / impl_library_;
        if (std::filesystem::exists(candidate1)) {
          impl_library_ = std::filesystem::canonical(candidate1).string();
        } else if (std::filesystem::exists(candidate2)) {
          impl_library_ = std::filesystem::canonical(candidate2).string();
        } else {
          RCLCPP_WARN(get_node()->get_logger(), "impl_library '%s' not found relative to package share; will pass original string to dlopen", impl_library_.c_str());
        }
      } catch(const std::exception& ex) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to resolve impl_library relative path: %s", ex.what());
      }
    }
  }

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
  // Initialize robot model owned by wrapper
  if(!robot_model_.init(urdf_xml_, ee_frame_hint)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RobotModel with provided URDF");
    return CallbackReturn::ERROR;
  }

  // Instantiate implementation only; defer initialize() until activation
  std::string load_err;
  if(!instantiateImplementation(load_err)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to instantiate implementation: %s", load_err.c_str());
    return CallbackReturn::ERROR;
  }
  if (impl_) { impl_->setRobotModel(static_cast<void*>(&robot_model_)); }
  RCLCPP_INFO(get_node()->get_logger(), "Instantiated control implementation library (initialization deferred): %s", impl_library_.c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception fetching robot description: %s", e.what());
    return CallbackReturn::ERROR;
  }


  return CallbackReturn::SUCCESS;
}

void GenericCartesianControllerWrapper::cartesian_command_callback(const compliant_controllers_msgs::msg::CartesianCommand::SharedPtr msg) {
  control::ControlCommand cmd(static_cast<size_t>(num_joints_));

  cmd.position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

  cmd.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                       msg->pose.orientation.y, msg->pose.orientation.z).normalized();

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
  RCLCPP_DEBUG(get_node()->get_logger(), "%s", oss.str().c_str());

  rt_cartesian_cmd_buffer_.writeFromNonRT(cmd);

}

CallbackReturn GenericCartesianControllerWrapper::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
 
  state_buffer_ = control::ControllerState(num_joints_);
  readControllerState(state_buffer_);
  robot_model_.update(state_buffer_.q);
  robot_model_.getPose(state_buffer_.position, state_buffer_.orientation);

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
  init_cmd.k_ns = Eigen::VectorXd::Zero(num_joints_);
  init_cmd.d_ns = Eigen::VectorXd::Zero(num_joints_);
  rt_cartesian_cmd_buffer_.writeFromNonRT(init_cmd);

  tau_out_.resize(num_joints_);
  impl_->step(init_cmd, state_buffer_, tau_out_, 0.0);
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCartesianControllerWrapper::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & ci : command_interfaces_) { ci.set_value(0.0); }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GenericCartesianControllerWrapper::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  readControllerState(state_buffer_);   // updates joint positions, velocities, and torques from hardware interface
  robot_model_.update(state_buffer_.q); // passes current joint positions to the robot model
  robot_model_.getPose(state_buffer_.position, state_buffer_.orientation); // updates end-effector pose in state buffer
                                                                           // based on calculations in the robot model

  // read latest command from RT buffer and pass it to the implementation
  const control::ControlCommand* latest_cmd = rt_cartesian_cmd_buffer_.readFromRT();
  // call step function of the controller implementation, returning control output torques
  impl_->step(*latest_cmd, state_buffer_, tau_out_, 0.001);

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_out_(i));
  }
  
  return controller_interface::return_type::OK;
}

bool GenericCartesianControllerWrapper::instantiateImplementation(std::string& err) {
  if (impl_) { destroyImplementation(); }

  if (impl_library_.empty()) { err = "impl_library parameter must be set (built-in removed)"; return false; }

  impl_handle_ = dlopen(impl_library_.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!impl_handle_) { err = dlerror(); return false; }

  // Unified simple factory C API (no versioning): create_controller / destroy_controller / controller_name
  auto create_fn = reinterpret_cast<CreateFn*>(dlsym(impl_handle_, "create_controller"));
  destroy_fn_ = reinterpret_cast<DestroyFn*>(dlsym(impl_handle_, "destroy_controller"));
  name_fn_ = reinterpret_cast<NameFn*>(dlsym(impl_handle_, "controller_name"));
  if (!create_fn || !destroy_fn_) {
    err = "Required symbols (create_controller, destroy_controller) missing";
    if (impl_handle_) { dlclose(impl_handle_); impl_handle_ = nullptr; }
    destroy_fn_ = nullptr; name_fn_ = nullptr; return false;
  }
  impl_ = create_fn();
  if(!impl_) { err = "Factory returned null"; if (impl_handle_) { dlclose(impl_handle_); impl_handle_=nullptr; } destroy_fn_=nullptr; name_fn_=nullptr; return false; }
  RCLCPP_INFO(get_node()->get_logger(), "Loaded controller impl '%s'", name_fn_?name_fn_():"<unknown>");
  // Provide robot model pointer prior to initialize so impl can seed if desired
  if (impl_) { impl_->setRobotModel(static_cast<void*>(&robot_model_)); }
  // Initialization deferred to controller on_activate()

  return true;
}

void GenericCartesianControllerWrapper::destroyImplementation() {
  if (impl_ && destroy_fn_) { destroy_fn_(impl_); }
  impl_ = nullptr; destroy_fn_ = nullptr; name_fn_ = nullptr;
  if (impl_handle_) { dlclose(impl_handle_); impl_handle_ = nullptr; }
}

} // namespace compliant_controllers

PLUGINLIB_EXPORT_CLASS(compliant_controllers::GenericCartesianControllerWrapper, controller_interface::ControllerInterface)
