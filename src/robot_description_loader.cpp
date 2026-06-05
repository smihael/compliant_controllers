#include <compliant_controllers/robot_description_loader.hpp>

#include <chrono>
#include <memory>

namespace compliant_controllers {

bool RobotDescriptionLoader::load(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  robot_description_node_ = node->get_parameter("robot_description_node").as_string();
  robot_description_param_ = node->get_parameter("robot_description_param").as_string();
  urdf_xml_.clear();
  source_description_.clear();

  try {
    urdf_xml_ = node->get_parameter("robot_description").as_string();
    if (!urdf_xml_.empty()) {
      source_description_ = "local parameter 'robot_description'";
      RCLCPP_INFO(node->get_logger(), "Got robot_description from local parameters");
      return true;
    }
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(node->get_logger(), "robot_description not available locally: %s", e.what());
  }

  auto parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node, robot_description_node_);
  if (!parameters_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(),
                 "Parameters service for node '%s' not available within timeout.",
                 robot_description_node_.c_str());
    return false;
  }

  try {
    auto future = parameters_client->get_parameters({robot_description_param_});
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(node->get_logger(),
                   "Timed out waiting for parameter '%s/%s'.",
                   robot_description_node_.c_str(),
                   robot_description_param_.c_str());
      return false;
    }

    auto results = future.get();
    if (results.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Parameter '%s' not returned.", robot_description_param_.c_str());
      return false;
    }
    if (results.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      RCLCPP_ERROR(node->get_logger(), "Parameter '%s' has wrong type.", robot_description_param_.c_str());
      return false;
    }

    urdf_xml_ = results.front().as_string();
    if (urdf_xml_.empty()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Received empty URDF string from '%s/%s'.",
                   robot_description_node_.c_str(),
                   robot_description_param_.c_str());
      return false;
    }

    source_description_ = robot_description_node_ + "/" + robot_description_param_;
    RCLCPP_INFO(node->get_logger(),
                "Got robot_description from remote node '%s'",
                robot_description_node_.c_str());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception fetching robot description: %s", e.what());
    return false;
  }
}

}  // namespace compliant_controllers
