#include <compliant_controllers/generic_cartesian_controller_wrapper.hpp>

namespace compliant_controllers {
void GenericCartesianControllerWrapper::setPayload(
    const std::shared_ptr<compliant_controllers_msgs::srv::SetPayload::Request> request,
    std::shared_ptr<compliant_controllers_msgs::srv::SetPayload::Response> response) {
  std::lock_guard<std::mutex> lock(payload_mutex_);
  response->success = false;
  if (!payload_configured_ || !payload_compensation_enabled_) {
    response->message = "Payload compensation is not configured and enabled";
    return;
  }
  try {
    // Validate before TF lookup; zero mass is a frame-independent clear operation.
    PayloadProperties::fromMessage(request->inertia, Eigen::Isometry3d::Identity());
    Eigen::Isometry3d T_EF = Eigen::Isometry3d::Identity();
    if (request->inertia.m > 0) {
      if (request->frame_id.empty()) throw std::invalid_argument("Positive mass requires frame_id");
      const auto it = target_frame_transforms_.find(request->frame_id);
      if (it != target_frame_transforms_.end()) {
        T_EF = it->second;
      } else {
        if (robot_frames_.count(request->frame_id))
          throw std::invalid_argument("Payload frame is not rigidly attached to the controlled frame");
        const auto tf = command_tf_buffer_->lookupTransform(
          robot_model_.endEffectorFrame(), request->frame_id, tf2::TimePointZero);
        const auto& q = tf.transform.rotation;
        const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-6)
          throw std::invalid_argument("Invalid TF rotation");
        T_EF.linear() = rotation.normalized().toRotationMatrix();
        const auto& t = tf.transform.translation;
        T_EF.translation() << t.x, t.y, t.z;
      }
    }
    const auto target = PayloadProperties::fromMessage(request->inertia, T_EF);
    rt_payload_buffer_.writeFromNonRT(target);
    response->success = true;
    response->message = target.mass == 0 ? "Payload clearing accepted" : "Payload replacement accepted";
  } catch (const std::exception& error) {
    response->message = error.what();
  }
}

bool GenericCartesianControllerWrapper::addPayloadCompensation(bool pose_ok) {
  if (!payload_compensation_enabled_) return true;
  if (!pose_ok || !state_buffer_.position.allFinite() ||
      !state_buffer_.orientation.coeffs().allFinite() ||
      std::abs(state_buffer_.orientation.norm() - 1.0) > 1e-6 ||
      !robot_model_.getJacobian(payload_jacobian_) || !payload_jacobian_.allFinite()) return false;
  const auto* target = rt_payload_buffer_.readFromRT();
  if (!target) return false;
  Eigen::Isometry3d T_BE = Eigen::Isometry3d::Identity();
  T_BE.linear() = state_buffer_.orientation.toRotationMatrix();
  T_BE.translation() = state_buffer_.position;
  payload_target_base_ = target->transformed(T_BE);
  if (!payload_target_base_.first_moment.allFinite() || !payload_target_base_.inertia.allFinite())
    return false;
  const auto wrench = payload_compensation_.step(*target, payload_compensation_alpha_,
                                                state_buffer_.orientation.toRotationMatrix());
  payload_torque_.noalias() = payload_jacobian_.transpose() * wrench;
  if (!payload_torque_.allFinite()) return false;
  tau_out_ += payload_torque_;
  return tau_out_.allFinite();
}
}  // namespace compliant_controllers
