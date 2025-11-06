#include <compliant_controllers/cartesian_impedance_impl.hpp>
#include <iostream>

using control::ControlCommand;

namespace compliant_controllers {

CartesianImpedanceImpl::CartesianImpedanceImpl(int num_joints) : num_joints_(num_joints) {
  J_.resize(6, num_joints_);
  pinv_Jt_.resize(num_joints_, 6); // will store pseudo-inverse of J^T (num_joints x 6)

  // Print in green using ANSI escape sequence
  std::cout << "\033[32mUsing " << kName << " compiled at " << __DATE__ << ", " << __TIME__ << "\033[0m" << std::endl;
}


Eigen::Vector3d CartesianImpedanceImpl::q_log(const Eigen::Quaterniond &q) const {
  Eigen::Quaterniond qn = q.normalized();
  double w = std::clamp(qn.w(), -1.0, 1.0);
  double angle = 2.0 * std::acos(w);
  double s = std::sqrt(1 - w * w);
  if (s < 1e-8 || angle < 1e-8) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d axis(qn.x() / s, qn.y() / s, qn.z() / s);
  return angle * axis / 2.0;
}

void CartesianImpedanceImpl::pseudoInverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_pinv, double tolerance) const {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tol = tolerance * std::max(M.cols(), M.rows()) * svd.singularValues().array().abs()(0);
  Eigen::VectorXd singular_inv = svd.singularValues();
  for (int i = 0; i < singular_inv.size(); ++i) {
    singular_inv(i) = (singular_inv(i) > tol) ? 1.0 / singular_inv(i) : 0.0;
  }
  M_pinv = svd.matrixV() * singular_inv.asDiagonal() * svd.matrixU().transpose();
}

bool CartesianImpedanceImpl::step(const ControlCommand& command,
                                  const control::ControllerState& current_state,
                                  Eigen::Ref<Eigen::VectorXd> control_output,
                                  double /*dt*/) {

  // Update model and retrieve Jacobian & pose
  robot_model_->update(current_state.q);
  robot_model_->getJacobian(J_);
  //robot_model_->getCoriolis(coriolis_);

  // Build error vector
  Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double,6,1>::Zero();
  error.head<3>() = current_state.position - command.position;

  Eigen::Vector3d rot_err = 2.0 * q_log(current_state.orientation.inverse() * command.orientation);
  error.tail<3>() = -current_state.orientation.toRotationMatrix() * rot_err;

  Eigen::Matrix<double,6,1> vel_error = J_*current_state.dq - command.velocity;

  // Pseudo inverse of J^T for nullspace projection (J: 6 x n) -> (J^T)^+ : n x 6
  pseudoInverse(J_.transpose(), pinv_Jt_);

  Eigen::VectorXd tau_task = J_.transpose() * (-command.stiffness * error - command.damping * vel_error);
  Eigen::VectorXd tau_ft_added = J_.transpose() * command.wrench;
  Eigen::VectorXd tau_nullspace = (Eigen::MatrixXd::Identity(num_joints_, num_joints_) - J_.transpose() * pinv_Jt_) *
      (command.k_ns.array() * (command.q_ns_des - current_state.q).array()).matrix();

  // CHECK!!
  if (tau_nullspace.norm() > TAU_NULLSPACE_MAX_) {
    tau_nullspace = tau_nullspace * TAU_NULLSPACE_MAX_ / tau_nullspace.norm();
  }

  if (control_output.size() == num_joints_) {
    //control_output = tau_task + tau_nullspace + coriolis + tau_ft_added;
    control_output = tau_task + tau_ft_added;
  }
  return true;
}

} // namespace compliant_controllers

// Export factory symbols for runtime loading
FACTORY_EXPORT_CONTROLLER(compliant_controllers::CartesianImpedanceImpl)
