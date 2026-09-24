#include <compliant_controllers/cartesian_impedance_impl.hpp>
#include <iostream>
#include <cmath>

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

  // Use the model state updated by the wrapper.
  if (!robot_model_->getJacobianAndPseudoInverse(J_, J_pinv_, 1e-6)) {
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

  control_output.noalias() = tau_task + tau_ft_added + tau_ns; // + coriolis
  return true;
}

} // namespace compliant_controllers

// Export factory symbols for runtime loading
FACTORY_EXPORT_CONTROLLER(compliant_controllers::CartesianImpedanceImpl)
