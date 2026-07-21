#include <compliant_controllers/cartesian_impedance_impl.hpp>
#include <iostream>
#include <cmath>

namespace {
bool allFinite(const Eigen::VectorXd& v) {
  return v.array().isFinite().all();
}

template <typename Derived>
bool allFiniteMat(const Eigen::MatrixBase<Derived>& m) {
  return m.array().isFinite().all();
}
}  // namespace

using control::ControlCommand;

namespace compliant_controllers {

CartesianImpedanceImpl::CartesianImpedanceImpl(int num_joints) : num_joints_(num_joints) {
  J_.resize(6, num_joints_);
  J_pinv_.resize(num_joints_, 6); 
  I_n_.resize(num_joints_, num_joints_);
  I_n_.setIdentity();

  // Print in green using ANSI escape sequence
  std::cout << "\033[32mUsing " << kName << " compiled at " << __DATE__ << ", " << __TIME__ << "\033[0m" << std::endl;
}


Eigen::Vector3d CartesianImpedanceImpl::q_log(const Eigen::Quaterniond &q) const {
  Eigen::Quaterniond qn = q.normalized();
  // q and -q encode the same orientation; use the shortest rotation.
  if (qn.w() < 0.0) {
    qn.coeffs() *= -1.0;
  }
  double w = std::clamp(qn.w(), -1.0, 1.0);
  double angle = 2.0 * std::acos(w);
  double s = std::sqrt(1 - w * w);
  if (s < 1e-8 || angle < 1e-8) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d axis(qn.x() / s, qn.y() / s, qn.z() / s);
  return angle * axis / 2.0;
}


bool CartesianImpedanceImpl::step(const ControlCommand& command,
                                  const control::ControllerState& current_state,
                                  Eigen::Ref<Eigen::VectorXd> control_output,
                                  double /*dt*/) {

  if (control_output.size() != num_joints_) {
    std::cerr << "[CartesianImpedanceImpl::step] control_output size mismatch: expected "
              << num_joints_ << ", got " << control_output.size() << std::endl;
    return false;
  }

  if (robot_model_ == nullptr) {
    std::cerr << "[CartesianImpedanceImpl] robot_model_ is null, skipping step" << std::endl;
    control_output.setZero();
    return false;
  }

  if (current_state.q.size() != num_joints_ || current_state.dq.size() != num_joints_ ||
      !allFinite(current_state.q) || !allFinite(current_state.dq) ||
      !allFiniteMat(command.position) || !allFiniteMat(command.velocity) ||
      !allFiniteMat(command.wrench) || !allFiniteMat(command.stiffness) ||
      !allFiniteMat(command.damping)) {
    std::cerr << "[CartesianImpedanceImpl::step] Invalid state/command input (size or non-finite)."
              << " q_size=" << current_state.q.size()
              << " dq_size=" << current_state.dq.size()
              << " expected=" << num_joints_ << std::endl;
    control_output.setZero();
    return false;
  }

  // Wrapper already updated robot model with current_state.q; fetch Jacobian and J^+.
  if (!robot_model_->getJacobianAndPseudoInverse(J_, J_pinv_, 1e-6)) {
    std::cerr << "[CartesianImpedanceImpl::step] Failed to get Jacobian/pseudo-inverse for current state." << std::endl;
    control_output.setZero();
    return false;
  }
  if (!allFiniteMat(J_) || !allFiniteMat(J_pinv_)) {
    std::cerr << "[CartesianImpedanceImpl::step] Non-finite Jacobian/pseudo-inverse detected." << std::endl;
    control_output.setZero();
    return false;
  }
  // robot_model_->getCoriolis(coriolis_);

  // Build error vector
  Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double,6,1>::Zero();
  error.head<3>() = current_state.position - command.position;

  Eigen::Vector3d rot_err = 2.0 * q_log(current_state.orientation.inverse() * command.orientation);
  error.tail<3>() = -current_state.orientation.toRotationMatrix() * rot_err;

  Eigen::Matrix<double,6,1> vel_error = J_*current_state.dq - command.velocity;


  Eigen::VectorXd tau_task = J_.transpose() * (-command.stiffness * error - command.damping * vel_error);
  Eigen::VectorXd tau_ft_added = J_.transpose() * command.wrench;
  

  Eigen::VectorXd tau_ns = Eigen::VectorXd::Zero(num_joints_);
  if (command.q_ns_des.size() == num_joints_ && command.k_ns.size() == num_joints_) {
    Eigen::VectorXd d_ns = Eigen::VectorXd::Zero(num_joints_);
    if (command.d_ns.size() == num_joints_) {
      d_ns = command.d_ns;
    }
    const Eigen::VectorXd tau_ns_raw =
      (command.k_ns.array() * (command.q_ns_des - current_state.q).array()).matrix()
      - (d_ns.array() * current_state.dq.array()).matrix();
    tau_ns = (I_n_ - J_.transpose() * J_pinv_.transpose()).lazyProduct(tau_ns_raw);
    double ns_norm = tau_ns.norm();
    if (ns_norm > TAU_NULLSPACE_MAX_) {
      tau_ns *= (TAU_NULLSPACE_MAX_ / ns_norm);
    }
  }

  if (control_output.size() == num_joints_) {
    control_output.noalias() = tau_task + tau_ft_added + tau_ns; // + coriolis
  }
  return true;
}

} // namespace compliant_controllers

// Export factory symbols for runtime loading
FACTORY_EXPORT_CONTROLLER(compliant_controllers::CartesianImpedanceImpl)
