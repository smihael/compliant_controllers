#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/inertia.hpp>
#include <stdexcept>

namespace compliant_controllers {

// All tensors are about the CoM. No parallel-axis shift is needed when changing axes.
struct PayloadProperties {
  double mass{0.0};
  Eigen::Vector3d first_moment{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};

  // Changes both origin and axes; first_moment / mass is the CoM position.
  PayloadProperties transformed(const Eigen::Isometry3d& T) const {
    PayloadProperties result;
    result.mass = mass;
    result.first_moment = T.linear() * first_moment + mass * T.translation();
    result.inertia = T.linear() * inertia * T.linear().transpose();
    return result;
  }

  static PayloadProperties fromMessage(const geometry_msgs::msg::Inertia& msg,
                                      const Eigen::Isometry3d& T_EF) {
    const Eigen::Vector3d com(msg.com.x, msg.com.y, msg.com.z);
    Eigen::Matrix3d I;
    I << msg.ixx, msg.ixy, msg.ixz, msg.ixy, msg.iyy, msg.iyz,
         msg.ixz, msg.iyz, msg.izz;
    if (!std::isfinite(msg.m) || msg.m < 0 || !com.allFinite() || !I.allFinite())
      throw std::invalid_argument("Mass must be finite and nonnegative; CoM and inertia must be finite");
    if (msg.m == 0 && !I.isZero(0.0))
      throw std::invalid_argument("Zero-mass payload must have zero inertia");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(I);
    const auto moments = solver.eigenvalues();
    const double tolerance = 1e-12 * std::max(1.0, I.norm());
    if (solver.info() != Eigen::Success || moments.minCoeff() < -tolerance ||
        moments[2] > moments[0] + moments[1] + tolerance)
      throw std::invalid_argument("Inertia must be positive semidefinite and satisfy principal-moment triangle inequalities");
    if (!T_EF.matrix().allFinite() ||
        !(T_EF.linear().transpose() * T_EF.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
        std::abs(T_EF.linear().determinant() - 1.0) > 1e-9)
      throw std::invalid_argument("Invalid payload frame transform");
    PayloadProperties result;
    result.mass = msg.m;
    result.first_moment = msg.m * (T_EF * com);
    result.inertia = T_EF.linear() * I * T_EF.linear().transpose();
    if (!result.first_moment.allFinite() || !result.inertia.allFinite())
      throw std::invalid_argument("Transformed mass properties overflowed");
    return result;
  }
};

class PayloadCompensation {
public:
  void reset() { mass_ = 0; first_moment_.setZero(); }
  Eigen::Matrix<double, 6, 1> step(const PayloadProperties& target, double alpha,
                                  const Eigen::Matrix3d& R_BE) {
    mass_ += alpha * (target.mass - mass_);
    first_moment_ += alpha * (target.first_moment - first_moment_);
    // Pinocchio's fixed-base model defaults to this gravity in model-base axes.
    const Eigen::Vector3d gravity(0, 0, -9.81);
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.head<3>() = -mass_ * gravity;
    wrench.tail<3>() = -(R_BE * first_moment_).cross(gravity);
    return wrench;
  }
private:
  double mass_{0};
  Eigen::Vector3d first_moment_{Eigen::Vector3d::Zero()};
};
}  // namespace compliant_controllers
