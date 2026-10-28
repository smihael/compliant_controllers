#pragma once

#include <compliant_controllers/compliance_frame.hpp>
#include <urdf/model.h>
#include <map>
#include <stdexcept>

namespace compliant_controllers {

// Configuration-only URDF lookup. RobotModel exposes only its controlled frame;
// fixed-joint composition is sufficient for every frame supported here, without
// a second model, per-cycle string lookup, or changes to the model/plugin ABI.
inline ComplianceFrame resolveComplianceFrame(
    const std::string& xml, const std::string& ee, const std::string& frame) {
  ComplianceFrame result;
  if (frame.empty()) return result;
  urdf::Model model;
  if (!model.initString(xml)) throw std::invalid_argument("Cannot parse robot URDF");
  const auto linkForFrame = [&](const std::string& name) {
    auto link = model.getLink(name);
    if (!link) {
      const auto joint = model.getJoint(name);
      if (joint) link = model.getLink(joint->child_link_name);
    }
    if (!link) throw std::invalid_argument("Unknown URDF link/joint frame '" + name + "'");
    return link;
  };
  const auto fixedAncestors = [&](const std::string& name) {
    std::map<std::string, Eigen::Isometry3d> transforms;
    auto link = linkForFrame(name);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    while (link) {
      transforms.emplace(link->name, T);
      const auto joint = link->parent_joint;
      if (!joint || joint->type != urdf::Joint::FIXED) break;
      const auto& origin = joint->parent_to_joint_origin_transform;
      Eigen::Isometry3d parent = Eigen::Isometry3d::Identity();
      parent.translation() << origin.position.x, origin.position.y, origin.position.z;
      parent.linear() = Eigen::Quaterniond(origin.rotation.w, origin.rotation.x,
                                           origin.rotation.y, origin.rotation.z).toRotationMatrix();
      T = parent * T;
      link = link->getParent();
    }
    return transforms;
  };
  const auto e = fixedAncestors(ee);
  const auto s = fixedAncestors(frame);
  for (const auto& [ancestor, T_AE] : e) {
    const auto it = s.find(ancestor);
    if (it != s.end()) {
      result.enabled = true;
      result.T_ES = T_AE.inverse() * it->second;
      return result;
    }
  }
  throw std::invalid_argument("compliance_frame '" + frame + "' is not rigidly attached to control_frame '" + ee + "'");
}
}  // namespace compliant_controllers
