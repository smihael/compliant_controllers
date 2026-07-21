#pragma once

// Archived parameter forwarding utilities.
// Not currently used but kept for future use with dynamically loaded
// controller libraries that consume ROS2 parameters.

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <control/AbstractController.hpp>

namespace compliant_controllers::parameter_forwarding {

inline std::optional<control::AbstractController::ParameterValue> parameterToValue(
  const rclcpp::Parameter& parameter) {
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return parameter.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return parameter.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return parameter.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return parameter.as_string();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return parameter.as_integer_array();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return parameter.as_double_array();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return parameter.as_string_array();
    default:
      return std::nullopt;
  }
}

inline void forwardAllParametersToImpl(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  control::AbstractController* impl,
  const std::optional<std::string>& deferred_string_parameter = std::nullopt) {
  if (node == nullptr || impl == nullptr) {
    return;
  }

  const auto listed = node->list_parameters({}, 10);
  std::string deferred_string_value;
  bool have_deferred_string_value{false};
  for (const auto& param_name : listed.names) {
    if (!node->has_parameter(param_name)) {
      continue;
    }

    const auto parameter = node->get_parameter(param_name);
    if (deferred_string_parameter.has_value() && param_name == *deferred_string_parameter &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      deferred_string_value = parameter.as_string();
      have_deferred_string_value = true;
      continue;
    }

    const auto value = parameterToValue(parameter);
    if (value.has_value()) {
      impl->setParameter(param_name, *value);
    }
  }

  if (deferred_string_parameter.has_value() && have_deferred_string_value) {
    impl->setParameter(*deferred_string_parameter, deferred_string_value);
  }
}

}  // namespace compliant_controllers::parameter_forwarding
