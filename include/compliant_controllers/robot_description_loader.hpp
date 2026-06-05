#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace compliant_controllers {

class RobotDescriptionLoader {
public:
  bool load(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  const std::string& urdfXml() const { return urdf_xml_; }
  const std::string& sourceDescription() const { return source_description_; }

private:
  std::string robot_description_node_{"robot_state_publisher"};
  std::string robot_description_param_{"robot_description"};
  std::string urdf_xml_;
  std::string source_description_;
};

}  // namespace compliant_controllers
