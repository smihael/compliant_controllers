#include <compliant_controllers/robot_model.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <optional>
#include <iostream>

namespace compliant_controllers {

struct RobotModel::Impl {
  pinocchio::Model model;
  pinocchio::Data data{model};
  pinocchio::FrameIndex ee_id{0};
  std::string ee_name;
  bool initialized{false};
  bool last_update_ok{false};
  int nj{0};
  Eigen::VectorXd q_cache; // sized at init

  bool build(const std::string &urdf_xml, const std::string &ee_hint) {
    try {
      pinocchio::urdf::buildModelFromXML(urdf_xml, model);
      data = pinocchio::Data(model);
      // choose ee frame
      if(!ee_hint.empty() && model.existFrame(ee_hint)) {
        ee_id = model.getFrameId(ee_hint);
        ee_name = ee_hint;
      } else {
        const std::vector<std::string> candidates{"tool0","ee_link","fr3_hand","panda_hand"};
        bool found=false;
        for(const auto & n: candidates){ if(model.existFrame(n)){ ee_id = model.getFrameId(n); ee_name = n; found=true; break; }}
        if(!found && !model.frames.empty()) { ee_id = model.frames.size()-1; ee_name = model.frames[ee_id].name; } // fallback last
      }
      nj = model.nv; // velocity dofs
      q_cache = Eigen::VectorXd::Zero(model.nq);
      initialized = true;
      last_update_ok = false;
      return true;
    } catch(const std::exception &e) {
      std::cerr << "RobotModel build failed: " << e.what() << std::endl;
      initialized = false;
      return false;
    }
  }

  bool update(const Eigen::Ref<const Eigen::VectorXd>& q_in) {
    if(!initialized) return false;
    if(q_in.size() < model.nq) return false;
    q_cache.head(model.nq) = q_in.head(model.nq);
    try {
      pinocchio::forwardKinematics(model, data, q_cache);
      pinocchio::computeJointJacobians(model, data, q_cache);
      pinocchio::updateFramePlacements(model, data);
      last_update_ok = true;
      return true;
    } catch(const std::exception &e) {
      last_update_ok = false;
      return false;
    }
  }

  bool pose(Eigen::Vector3d &p, Eigen::Quaterniond &q) const {
    if(!(initialized && last_update_ok)) return false;
    const auto & placement = data.oMf[ee_id];
    p = placement.translation();
    q = Eigen::Quaterniond(placement.rotation()).normalized();
    return true;
  }

  bool jacobian(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out) {
    if(!(initialized && last_update_ok)) return false;
    // pinocchio requires non-const data reference for getFrameJacobian
    Eigen::Matrix<double,6,Eigen::Dynamic> J = pinocchio::getFrameJacobian(model, data, ee_id, pinocchio::LOCAL_WORLD_ALIGNED);
    if(J_out.cols() < J.cols()) {
      return false; // insufficient provided cols
    }
    J_out.leftCols(J.cols()) = J;
    return true;
  }
};

RobotModel::RobotModel() : impl_(std::make_unique<Impl>()) {}
RobotModel::~RobotModel() = default;
RobotModel::RobotModel(RobotModel&&) noexcept = default;
RobotModel& RobotModel::operator=(RobotModel&&) noexcept = default;

bool RobotModel::init(const std::string &urdf_xml, const std::string &ee_hint) {
  if(!impl_) impl_ = std::make_unique<Impl>();
  bool ok = impl_->build(urdf_xml, ee_hint);
  nj = ok ? impl_->nj : 0;
  return ok;
}

bool RobotModel::update(const Eigen::Ref<const Eigen::VectorXd>& q){
  if(!impl_) return false;
  return impl_->update(q);
}

bool RobotModel::getPose(Eigen::Vector3d &position, Eigen::Quaterniond &orientation) const {
  if(!impl_) return false;
  return impl_->pose(position, orientation);
}

bool RobotModel::getJacobian(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out) {
  if(!impl_) return false;
  return impl_->jacobian(J_out);
}

bool RobotModel::valid() const {
  return impl_ && impl_->initialized && impl_->last_update_ok;
}

// Optional accessor for EE frame name
std::string RobotModel::endEffectorFrame() const {
  return impl_ ? impl_->ee_name : std::string();
}

} // namespace compliant_controllers
