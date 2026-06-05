#pragma once

#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace compliant_controllers::parameter_utils {

inline bool get_optional_bool(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                              const std::string& name,
                              bool fallback) {
  try {
    return node->get_parameter(name).as_bool();
  } catch (const std::exception&) {
    return fallback;
  }
}

inline double get_optional_double(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                  const std::string& name,
                                  double fallback) {
  try {
    return node->get_parameter(name).as_double();
  } catch (const std::exception&) {
    return fallback;
  }
}

inline std::vector<double> get_optional_double_array(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name) {
  try {
    return node->get_parameter(name).as_double_array();
  } catch (const std::exception&) {
    return {};
  }
}

inline std::string get_optional_string(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                       const std::string& name,
                                       const std::string& fallback) {
  try {
    return node->get_parameter(name).as_string();
  } catch (const std::exception&) {
    return fallback;
  }
}

}  // namespace compliant_controllers::parameter_utils
