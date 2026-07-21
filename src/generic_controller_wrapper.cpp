#include <compliant_controllers/generic_controller_wrapper.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <dlfcn.h>

namespace compliant_controllers {

controller_interface::InterfaceConfiguration GenericControllerWrapper::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration GenericControllerWrapper::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

void GenericControllerWrapper::configureLogging(const std::string& wrapper_name) {
  if (std::getenv("CONTROLLER_DEBUG") != nullptr) {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(logger_, "CONTROLLER_DEBUG is set; wrapper logger level set to DEBUG.");
  }

  RCLCPP_INFO_STREAM(logger_,
                     "\033[32m" << wrapper_name << " initialized; compiled at "
                                << __DATE__ << ", " << __TIME__ << "\033[0m");
}

bool GenericControllerWrapper::instantiateImplementation(std::string& err) {
  if (impl_) {
    destroyImplementation();
  }

  LoadedControllerLibrary lib;
  if (!loadControllerLibrary(lib, err)) {
    return false;
  }

  impl_handle_ = lib.handle;
  impl_ = lib.impl;
  destroy_fn_ = lib.destroy_fn;
  name_fn_ = lib.name_fn;

  if (impl_) {
    impl_->setRobotModel(static_cast<void*>(&robot_model_));
  }

  RCLCPP_INFO(logger_, "Loaded controller impl '%s' with %d joints",
              name_fn_ ? name_fn_() : "<unknown>", num_joints_);
  return true;
}

void GenericControllerWrapper::destroyImplementation() {
  LoadedControllerLibrary lib;
  lib.handle = impl_handle_;
  lib.impl = impl_;
  lib.destroy_fn = destroy_fn_;
  lib.name_fn = name_fn_;
  unloadControllerLibrary(lib);
  impl_handle_ = lib.handle;
  impl_ = lib.impl;
  destroy_fn_ = lib.destroy_fn;
  name_fn_ = lib.name_fn;
}

void GenericControllerWrapper::diagnostic_log_filter_tag_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  diagnostic_logger_.setLogFilterTag(msg->data);
}

void GenericControllerWrapper::readControllerState(control::ControllerState& out) {
  for (int i = 0; i < num_joints_; ++i) {
    out.q(i) = state_interfaces_[3 * i].get_value();
    out.dq(i) = state_interfaces_[3 * i + 1].get_value();
    out.tau(i) = state_interfaces_[3 * i + 2].get_value();
  }
}

void GenericControllerWrapper::sanitizeTorqueOutput() {
  constexpr double kTauDeadband = 1e-9;
  constexpr double kTauAbsMaxWrite = 120.0;
  for (int i = 0; i < num_joints_; ++i) {
    auto& tau = tau_out_(i);
    if (!std::isfinite(tau)) {
      tau = 0.0;
      continue;
    }
    if (std::abs(tau) < kTauDeadband) {
      tau = 0.0;
    }
    tau = std::clamp(tau, -kTauAbsMaxWrite, kTauAbsMaxWrite);
  }
}

void GenericControllerWrapper::writeTorqueOutput() {
  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_out_(i));
  }
}

void GenericControllerWrapper::writeZeroTorques() {
  if (tau_out_.size() != num_joints_) {
    tau_out_.setZero(num_joints_);
  } else {
    tau_out_.setZero();
  }
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
}

std::string GenericControllerWrapper::resolveSharedLibraryPath(const std::string& library_path) const {
  if (library_path.empty() || std::filesystem::path(library_path).is_absolute()) {
    return library_path;
  }

  try {
    const auto prefix = ament_index_cpp::get_package_prefix("compliant_controllers");
    const std::filesystem::path candidate = std::filesystem::path(prefix) / "lib" / library_path;
    if (std::filesystem::exists(candidate)) {
      return std::filesystem::canonical(candidate).string();
    }
    RCLCPP_WARN(logger_,
                "impl_library '%s' not found at '%s'; will pass original string to dlopen",
                library_path.c_str(), candidate.c_str());
  } catch (const std::exception& ex) {
    RCLCPP_WARN(logger_, "Failed to resolve impl_library path: %s", ex.what());
  }

  return library_path;
}

bool GenericControllerWrapper::loadControllerLibrary(LoadedControllerLibrary& out, std::string& err) {
  out = {};
  impl_library_ = resolveSharedLibraryPath(impl_library_);
  if (impl_library_.empty()) {
    err = "impl_library parameter must be set";
    return false;
  }

  using CreateFn = control::AbstractController* (*)(int);

  out.handle = dlopen(impl_library_.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!out.handle) {
    err = dlerror();
    return false;
  }

  const auto create_fn = reinterpret_cast<CreateFn>(dlsym(out.handle, "create_controller"));
  out.destroy_fn = reinterpret_cast<DestroyFn*>(dlsym(out.handle, "destroy_controller"));
  out.name_fn = reinterpret_cast<NameFn*>(dlsym(out.handle, "controller_name"));
  if (!create_fn || !out.destroy_fn) {
    err = "Required symbols (create_controller, destroy_controller) missing";
    dlclose(out.handle);
    out = {};
    return false;
  }

  out.impl = create_fn(num_joints_);
  if (!out.impl) {
    err = "Factory returned null";
    dlclose(out.handle);
    out = {};
    return false;
  }

  return true;
}

void GenericControllerWrapper::unloadControllerLibrary(LoadedControllerLibrary& lib) {
  if (lib.impl && lib.destroy_fn) {
    lib.destroy_fn(lib.impl);
  }
  lib.impl = nullptr;
  lib.destroy_fn = nullptr;
  lib.name_fn = nullptr;
  if (lib.handle) {
    dlclose(lib.handle);
    lib.handle = nullptr;
  }
}

}  // namespace compliant_controllers
