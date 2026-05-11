#include <compliant_controllers/generic_cartesian_controller_wrapper.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
// dlopen for external implementations
#include <dlfcn.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <std_msgs/msg/string.hpp>
#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>

namespace compliant_controllers {

namespace {
bool get_optional_bool(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                       const std::string& name,
                       bool fallback) {
  try {
    return node->get_parameter(name).as_bool();
  } catch (const std::exception&) {
    return fallback;
  }
}

double get_optional_double(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                           const std::string& name,
                           double fallback) {
  try {
    return node->get_parameter(name).as_double();
  } catch (const std::exception&) {
    return fallback;
  }
}

std::vector<double> get_optional_double_array(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                              const std::string& name) {
  try {
    return node->get_parameter(name).as_double_array();
  } catch (const std::exception&) {
    return {};
  }
}
}  // namespace

controller_interface::InterfaceConfiguration GenericCartesianControllerWrapper::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  if (use_named_joints_ && !joint_names_.empty()) {
    // Use explicitly provided joint names (e.g., for UR robots)
    for (const auto& joint_name : joint_names_) {
      config.names.push_back(joint_name + "/effort");
    }
  } else {
    // Fall back to default arm_id_joint{i} naming (Franka style)
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/effort");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration GenericCartesianControllerWrapper::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  if (use_named_joints_ && !joint_names_.empty()) {
    // Use explicitly provided joint names
    for (const auto& joint_name : joint_names_) {
      config.names.push_back(joint_name + "/position");
      config.names.push_back(joint_name + "/velocity");
      config.names.push_back(joint_name + "/effort");
    }
  } else {
    // Fall back to default arm_id_joint{i} naming
    for (int i = 1; i <= num_joints_; ++i) {
      config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/position");
      config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/velocity");
      config.names.push_back(arm_id_ + std::string("_joint") + std::to_string(i) + "/effort");
    }
  }

  return config;
}

void GenericCartesianControllerWrapper::readControllerState(control::ControllerState& out) {
  for (int i = 0; i < num_joints_; ++i) {
    out.q(i)   = state_interfaces_[3*i].get_value();
    out.dq(i)  = state_interfaces_[3*i + 1].get_value();
    out.tau(i) = state_interfaces_[3*i + 2].get_value();
  }
}

CallbackReturn GenericCartesianControllerWrapper::on_init() {
  // Cartesian command subscription (non RT -> pushes into realtime buffer)
  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  cartesian_command_sub_ = get_node()->create_subscription<compliant_controllers_msgs::msg::CartesianCommand>(
    "cartesian_command", command_qos,
    std::bind(&GenericCartesianControllerWrapper::cartesian_command_callback, this, std::placeholders::_1));

    // if CONTROLLER_DEBUG env is set, then set logger to DEBUG level
    if (std::getenv("CONTROLLER_DEBUG") != nullptr) {
      get_node()->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "\033[32mUsing GenericCartesianControllerWrapper compiled at " << __DATE__ << ", " << __TIME__ << "\033[0m");

  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCartesianControllerWrapper::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  // The following parameters are expected to be declared externally (e.g., via launch or YAML):

  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  init_k_pos_ = get_node()->get_parameter("init_k_pos").as_double();
  init_k_ori_ = get_node()->get_parameter("init_k_ori").as_double();

  // Optional gravity compensation parameter
  try {
    add_gravity_compensation_ = get_node()->get_parameter("add_gravity_compensation").as_bool();
    RCLCPP_INFO(get_node()->get_logger(), "Gravity compensation: %s", add_gravity_compensation_ ? "enabled" : "disabled");
  } catch (const std::exception& e) {
    add_gravity_compensation_ = false;
    RCLCPP_DEBUG(get_node()->get_logger(), "No 'add_gravity_compensation' parameter provided, defaulting to disabled");
  }

  // Check if explicit joint names are provided (e.g., for UR robots)
  try {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (!joint_names_.empty()) {
      use_named_joints_ = true;
      num_joints_ = joint_names_.size();
      RCLCPP_INFO(get_node()->get_logger(), "Using %zu explicitly named joints", joint_names_.size());
      RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Joint list: " << [this]() {
        std::ostringstream oss;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          oss << joint_names_[i];
          if (i + 1 < joint_names_.size()) oss << ", ";
        }
        return oss.str();
      }());
    }
  } catch (const std::exception& e) {
    // "joints" parameter not provided, will use default arm_id_joint{i} naming
    RCLCPP_DEBUG(get_node()->get_logger(), "No explicit 'joints' parameter provided, using default arm_id_joint{i} naming");
  }

  // Update configured remote parameter identifiers
  robot_description_node_ = get_node()->get_parameter("robot_description_node").as_string();
  robot_description_param_ = get_node()->get_parameter("robot_description_param").as_string();
  try {
    end_effector_profile_node_ = get_node()->get_parameter("end_effector_profile_node").as_string();
  } catch (const std::exception&) {
    end_effector_profile_node_.clear();
  }
  readEndEffectorLoadParameters();

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

  // Attempt to get robot_description - try local parameter first, then remote node
  std::string ee_frame_hint = get_node()->get_parameter("ee_frame").as_string();
  
  // Try to get robot_description from local parameters first (Gazebo sets this)
  bool got_urdf = false;
  try {
    urdf_xml_ = get_node()->get_parameter("robot_description").as_string();
    if (!urdf_xml_.empty()) {
      got_urdf = true;
      urdf_received_.store(true);
      RCLCPP_INFO(get_node()->get_logger(), "Got robot_description from local parameters");
    }
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(get_node()->get_logger(), "robot_description not available locally: %s", e.what());
  }

  // If not available locally, try to fetch from remote node
  if (!got_urdf) {
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
      RCLCPP_INFO(get_node()->get_logger(), "Got robot_description from remote node '%s'", robot_description_node_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception fetching robot description: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  // Initialize robot model owned by wrapper
  if(!robot_model_.init(urdf_xml_, ee_frame_hint, joint_names_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RobotModel with provided URDF");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Robot model initialized. ee_frame=%s controlled_dofs=%d", robot_model_.endEffectorFrame().c_str(), robot_model_.dofs());

  // Instantiate implementation only; defer initialize() until activation
  std::string load_err;
  if(!instantiateImplementation(load_err)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to instantiate implementation: %s", load_err.c_str());
    return CallbackReturn::ERROR;
  }
  if (impl_) { impl_->setRobotModel(static_cast<void*>(&robot_model_)); }
  RCLCPP_INFO(get_node()->get_logger(), "Instantiated control implementation library (initialization deferred): %s", impl_library_.c_str());

  return CallbackReturn::SUCCESS;
}

void GenericCartesianControllerWrapper::readEndEffectorLoadParameters() {
  compensate_end_effector_load_ =
    get_optional_bool(get_node(), "compensate_end_effector_load", false);
  load_gravity_acceleration_ =
    get_optional_double(get_node(), "end_effector_profile.load.gravity_acceleration", 9.80665);
  bool loaded_from_local_params = false;
  try {
    load_mass_ = get_node()->get_parameter("end_effector_profile.load.mass").as_double();
    loaded_from_local_params = true;
  } catch (const std::exception&) {
    load_mass_ = 0.0;
  }

  const auto com = get_optional_double_array(get_node(), "end_effector_profile.load.center_of_mass");
  if (com.size() == 3) {
    load_center_of_mass_ << com[0], com[1], com[2];
    loaded_from_local_params = true;
  }

  const auto inertia = get_optional_double_array(get_node(), "end_effector_profile.load.inertia");
  if (inertia.size() == 9) {
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>> inertia_map(inertia.data());
    load_inertia_ = inertia_map;
    loaded_from_local_params = true;
  }

  if (!end_effector_profile_node_.empty() &&
      fetchEndEffectorLoadParametersFromNode(end_effector_profile_node_)) {
    loaded_from_local_params = true;
  }

  compensate_end_effector_load_ =
    compensate_end_effector_load_ && std::isfinite(load_mass_) && load_mass_ > 0.0;

  if (compensate_end_effector_load_) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "End-effector load compensation enabled: mass=" << load_mass_
                       << " kg, COM=[" << load_center_of_mass_.transpose()
                       << "]");
  } else if (loaded_from_local_params) {
    RCLCPP_INFO(get_node()->get_logger(),
                "End-effector profile loaded, but load compensation is disabled or mass is zero.");
  } else {
    RCLCPP_DEBUG(get_node()->get_logger(), "No end-effector load profile parameters found.");
  }
}

bool GenericCartesianControllerWrapper::fetchEndEffectorLoadParametersFromNode(const std::string& node_name) {
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), node_name);
  bool service_ready = false;
  for (int attempt = 0; attempt < 20; ++attempt) {
    if (parameters_client->wait_for_service(std::chrono::milliseconds(500))) {
      service_ready = true;
      break;
    }
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Waiting for end-effector profile parameter node '%s'...",
                         node_name.c_str());
  }
  if (!service_ready) {
    RCLCPP_WARN(get_node()->get_logger(),
                "End-effector profile parameter node '%s' is not available.", node_name.c_str());
    return false;
  }

  const std::vector<std::string> names{
    "end_effector_profile.load.mass",
    "end_effector_profile.load.center_of_mass",
    "end_effector_profile.load.inertia",
    "end_effector_profile.load.gravity_acceleration",
    "compensate_end_effector_load",
  };

  try {
    auto future = parameters_client->get_parameters(names);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Timed out reading end-effector profile parameters from '%s'.", node_name.c_str());
      return false;
    }

    bool found = false;
    for (const auto& parameter : future.get()) {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        continue;
      }
      found = true;
      if (parameter.get_name() == "end_effector_profile.load.mass" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        load_mass_ = parameter.as_double();
      } else if (parameter.get_name() == "end_effector_profile.load.center_of_mass" &&
                 parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
                 parameter.as_double_array().size() == 3) {
        const auto values = parameter.as_double_array();
        load_center_of_mass_ << values[0], values[1], values[2];
      } else if (parameter.get_name() == "end_effector_profile.load.inertia" &&
                 parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
                 parameter.as_double_array().size() == 9) {
        const auto values = parameter.as_double_array();
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>> inertia_map(values.data());
        load_inertia_ = inertia_map;
      } else if (parameter.get_name() == "end_effector_profile.load.gravity_acceleration" &&
                 parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        load_gravity_acceleration_ = parameter.as_double();
      } else if (parameter.get_name() == "compensate_end_effector_load" &&
                 parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        compensate_end_effector_load_ = parameter.as_bool();
      }
    }
    return found;
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Exception reading end-effector profile parameters from '%s': %s",
                node_name.c_str(), e.what());
    return false;
  }
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
  impl_ = create_fn(num_joints_);
  if(!impl_) { err = "Factory returned null"; if (impl_handle_) { dlclose(impl_handle_); impl_handle_=nullptr; } destroy_fn_=nullptr; name_fn_=nullptr; return false; }
  RCLCPP_INFO(get_node()->get_logger(), "Loaded controller impl '%s' with %d joints", name_fn_?name_fn_():"<unknown>", num_joints_);
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

CallbackReturn GenericCartesianControllerWrapper::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
 
  state_buffer_ = control::ControllerState(num_joints_);
  readControllerState(state_buffer_);
  robot_model_.update(state_buffer_.q);
  robot_model_.getPose(state_buffer_.position, state_buffer_.orientation);
  active_since_ = get_node()->get_clock()->now();

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

  tau_out_.resize(num_joints_);
  impl_->step(init_cmd, state_buffer_, tau_out_, 0.001);

  RCLCPP_INFO(get_node()->get_logger(), "Controller wrapper activated, calling step() from %s", name_fn_?name_fn_():"<unknown>");
  RCLCPP_INFO(get_node()->get_logger(), "Command interfaces: expected=%d actual=%zu", num_joints_, command_interfaces_.size());
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    RCLCPP_INFO(get_node()->get_logger(), "  cmd_if[%zu]: %s", i, command_interfaces_[i].get_name().c_str());
  }
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Initial q: " << state_buffer_.q.transpose() << " | tau: " << state_buffer_.tau.transpose() << " | ee: [" << state_buffer_.position.transpose() << "]");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenericCartesianControllerWrapper::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & ci : command_interfaces_) { ci.set_value(0.0); }
  return CallbackReturn::SUCCESS;
}

void GenericCartesianControllerWrapper::cartesian_command_callback(const compliant_controllers_msgs::msg::CartesianCommand::SharedPtr msg) {

  const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  if (!has_stamp) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Dropping CartesianCommand with invalid (zero) header timestamp.");
    return;
  }

  const rclcpp::Time msg_stamp(msg->header.stamp, RCL_ROS_TIME);
  const rclcpp::Time now = get_node()->get_clock()->now();
  if (msg_stamp < active_since_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Dropping CartesianCommand stamped before controller activation (%.3fs before activation).",
                         (active_since_ - msg_stamp).seconds());
    return;
  }
  if (msg_stamp < now - rclcpp::Duration::from_seconds(0.5)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
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
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "CartesianCommand header.frame_id is empty; expected controlled frame '%s'.",
                           expected_frame.c_str());
    } else if (message_frame != expected) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Dropping CartesianCommand for frame '%s'; controller frame is '%s'.",
                           msg->header.frame_id.c_str(), expected_frame.c_str());
      return;
    }
  }

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
  // oss << "  position: [" << cmd.position.x() << ", " << cmd.position.y() << ", " << cmd.position.z() << "]\n";
  // oss << "  orientation: [" << cmd.orientation.w() << ", " << cmd.orientation.x() << ", " << cmd.orientation.y() << ", " << cmd.orientation.z() << "]\n";
  // oss << "  velocity: " << cmd.velocity.transpose() << "\n";
  // oss << "  wrench: " << cmd.wrench.transpose() << "\n";
  // oss << "  stiffness:\n" << cmd.stiffness << "\n";
  // oss << "  damping:\n" << cmd.damping << "\n";
  // oss << "  nullspace desired position: " << cmd.q_ns_des.transpose() << "\n";
  // oss << "  nullspace stiffness: " << cmd.k_ns.transpose() << "\n";
  // oss << "  nullspace damping: " << cmd.d_ns.transpose() << "\n";
  // oss << "  torque feedforward: " << cmd.tau_ff.transpose() << "\n";  
  RCLCPP_DEBUG(get_node()->get_logger(), "%s", oss.str().c_str());

  rt_cartesian_cmd_buffer_.writeFromNonRT(cmd);

}

controller_interface::return_type GenericCartesianControllerWrapper::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  readControllerState(state_buffer_);   // updates joint positions, velocities, and torques from hardware interface
  const bool model_ok = robot_model_.update(state_buffer_.q); // passes current joint positions to the robot model
  if (!model_ok) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "RobotModel update failed; writing zero torque for this cycle.");
    tau_out_.setZero();
    for (int i = 0; i < num_joints_; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  const bool pose_ok = robot_model_.getPose(state_buffer_.position, state_buffer_.orientation); // updates end-effector pose in state buffer
  if (!pose_ok) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "RobotModel pose update failed; keeping previous EE state.");
  }
  // read latest command from RT buffer and pass it to the implementation
  const control::ControlCommand* latest_cmd = rt_cartesian_cmd_buffer_.readFromRT();
  // call step function of the controller implementation, returning control output torques
  impl_->step(*latest_cmd, state_buffer_, tau_out_, 0.001);

  // Add gravity compensation if enabled (UR robots need this, Franka does it in hardware)
  if (add_gravity_compensation_) {
    Eigen::VectorXd g = Eigen::VectorXd::Zero(num_joints_);
    if (robot_model_.getGravity(g)) {
      if (!g.array().isFinite().all()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "Non-finite gravity torque detected; disabling gravity compensation.");
        add_gravity_compensation_ = false;
      } else {
        tau_out_ += g;
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Gravity vector unavailable; skipping gravity compensation this cycle.");
    }
  }
  addEndEffectorLoadCompensation();

  // Sanitize denormal/near-zero values and cap extreme torques before write.
  constexpr double kTauDeadband = 1e-9;
  constexpr double kTauAbsMaxWrite = 120.0;
  for (int i = 0; i < num_joints_; ++i) {
    double &tau_i = tau_out_(i);
    if (!std::isfinite(tau_i)) {
      tau_i = 0.0;
    } else {
      if (std::abs(tau_i) < kTauDeadband) {
        tau_i = 0.0;
      }
      if (tau_i > kTauAbsMaxWrite) {
        tau_i = kTauAbsMaxWrite;
      } else if (tau_i < -kTauAbsMaxWrite) {
        tau_i = -kTauAbsMaxWrite;
      }
    }
  }

  //tau_out_.setZero();
  for (int i = 0; i < num_joints_; ++i) {
    const double tau_i = tau_out_(i);
    if (!std::isfinite(tau_i)) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Non-finite torque detected at joint %d; forcing zero write.", i);
      command_interfaces_[i].set_value(0.0);
    } else {
      command_interfaces_[i].set_value(tau_i);
    }
  }

  return controller_interface::return_type::OK;
}

void GenericCartesianControllerWrapper::addEndEffectorLoadCompensation() {
  if (!compensate_end_effector_load_) {
    return;
  }

  if (load_jacobian_.cols() != num_joints_) {
    load_jacobian_.resize(6, num_joints_);
  }
  load_jacobian_.setZero();
  if (!robot_model_.getJacobian(load_jacobian_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "End-effector load compensation skipped because Jacobian is unavailable.");
    return;
  }

  const Eigen::Vector3d com_world =
    state_buffer_.orientation.toRotationMatrix() * load_center_of_mass_;
  const Eigen::Vector3d gravity_force_world(
    0.0, 0.0, load_mass_ * load_gravity_acceleration_);
  load_wrench_.head<3>() = gravity_force_world;
  load_wrench_.tail<3>() = com_world.cross(gravity_force_world);

  const Eigen::VectorXd tau_load = load_jacobian_.transpose() * load_wrench_;
  if (!tau_load.array().isFinite().all()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Non-finite end-effector load torque detected; skipping compensation.");
    return;
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                               "End-effector load compensation: wrench=["
                               << load_wrench_.transpose() << "], tau=["
                               << tau_load.transpose() << "]");
  tau_out_ += tau_load;
}


} // namespace compliant_controllers

PLUGINLIB_EXPORT_CLASS(compliant_controllers::GenericCartesianControllerWrapper, controller_interface::ControllerInterface)
