#include <compliant_controllers/joint_impedance_impl.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using control::ControlCommand;

namespace {
bool all_finite(const Eigen::VectorXd& v) {
  return v.array().isFinite().all();
}
}  // namespace

namespace compliant_controllers {

JointImpedanceImpl::JointImpedanceImpl(int num_joints) : num_joints_(num_joints) {
  coriolis_.setZero(num_joints_);
  tau_calculated_.setZero(num_joints_);
  tau_saturated_.setZero(num_joints_);
  tau_last_commanded_.setZero(num_joints_);
  q_d_.setZero(num_joints_);
  dq_d_.setZero(num_joints_);
  k_gains_.setZero(num_joints_);
  d_gains_.setZero(num_joints_);
  q_d_target_.setZero(num_joints_);
  dq_d_target_.setZero(num_joints_);
  k_gains_target_.setZero(num_joints_);
  d_gains_target_.setZero(num_joints_);
  feed_forward_q_target_.setZero(num_joints_);

  std::cout << "\033[32mUsing " << kName << " compiled at " << __DATE__ << ", "
            << __TIME__ << "\033[0m" << std::endl;
}

bool JointImpedanceImpl::validInput(const ControlCommand& command,
                                    const control::ControllerState& current_state,
                                    const Eigen::Ref<const Eigen::VectorXd>& control_output) const {
  return control_output.size() == num_joints_ &&
         current_state.q.size() == num_joints_ &&
         current_state.dq.size() == num_joints_ &&
         command.joint_position.size() == num_joints_ &&
         command.joint_velocity.size() == num_joints_ &&
         command.joint_stiffness.size() == num_joints_ &&
         command.joint_damping.size() == num_joints_ &&
         command.joint_torque_ff.size() == num_joints_ &&
         all_finite(current_state.q) &&
         all_finite(current_state.dq) &&
         all_finite(command.joint_position) &&
         all_finite(command.joint_velocity) &&
         all_finite(command.joint_stiffness) &&
         all_finite(command.joint_damping) &&
         all_finite(command.joint_torque_ff);
}

void JointImpedanceImpl::initializeTargets(const ControlCommand& command) {
  if (initialized_) {
    return;
  }
  q_d_ = command.joint_position;
  dq_d_ = command.joint_velocity;
  k_gains_ = command.joint_stiffness;
  d_gains_ = command.joint_damping;
  q_d_target_ = command.joint_position;
  dq_d_target_ = command.joint_velocity;
  k_gains_target_ = command.joint_stiffness;
  d_gains_target_ = command.joint_damping;
  feed_forward_q_target_ = command.joint_torque_ff;
  tau_last_commanded_.setZero();
  initialized_ = true;
}

void JointImpedanceImpl::filterTargets(const ControlCommand& command) {
  q_d_target_ = command.joint_position;
  dq_d_target_ = command.joint_velocity;
  k_gains_target_ = command.joint_stiffness;
  d_gains_target_ = command.joint_damping;
  feed_forward_q_target_ = command.joint_torque_ff;

  const double alpha = std::clamp(filter_alpha_, 0.0, 1.0);
  const double one_minus_alpha = 1.0 - alpha;
  k_gains_ = one_minus_alpha * k_gains_target_ + alpha * k_gains_;
  d_gains_ = one_minus_alpha * d_gains_target_ + alpha * d_gains_;
  q_d_ = one_minus_alpha * q_d_target_ + alpha * q_d_;
  dq_d_ = one_minus_alpha * dq_d_target_ + alpha * dq_d_;
}

void JointImpedanceImpl::saturateTorqueRate(const Eigen::Ref<const Eigen::VectorXd>& tau_desired,
                                            Eigen::Ref<Eigen::VectorXd> tau_saturated) {
  for (int i = 0; i < num_joints_; ++i) {
    const double diff = tau_desired(i) - tau_last_commanded_(i);
    tau_saturated(i) = tau_last_commanded_(i) + std::clamp(diff, -max_tau_delta_, max_tau_delta_);
  }
}

bool JointImpedanceImpl::step(const ControlCommand& command,
                              const control::ControllerState& current_state,
                              Eigen::Ref<Eigen::VectorXd> control_output,
                              double /*dt*/) {
  if (!validInput(command, current_state, control_output)) {
    std::cerr << "[JointImpedanceImpl::step] Invalid state or command dimensions/input." << std::endl;
    control_output.setZero();
    return false;
  }
  if (robot_model_ == nullptr || !robot_model_->getCoriolis(current_state.dq, coriolis_)) {
    coriolis_.setZero();
  }

  initializeTargets(command);

  tau_calculated_ = coriolis_
    + (k_gains_.array() * (q_d_ - current_state.q).array()).matrix()
    + (d_gains_.array() * (dq_d_ - current_state.dq).array()).matrix()
    + feed_forward_q_target_;

  saturateTorqueRate(tau_calculated_, tau_saturated_);
  control_output = tau_saturated_;
  tau_last_commanded_ = tau_saturated_;

  filterTargets(command);
  return true;
}

}  // namespace compliant_controllers

FACTORY_EXPORT_CONTROLLER(compliant_controllers::JointImpedanceImpl)
