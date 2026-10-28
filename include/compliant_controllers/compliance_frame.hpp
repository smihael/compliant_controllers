#pragma once

#include <Eigen/Geometry>

namespace compliant_controllers {

// All spatial vectors use [linear; angular], expressed in robot-base axes.
// T_ES maps coordinates from the rigid compliance frame S to controlled frame E.
struct ComplianceFrame {
  bool enabled{false};
  Eigen::Isometry3d T_ES{Eigen::Isometry3d::Identity()};

  Eigen::Vector3d offset(const Eigen::Quaterniond& orientation) const {
    return orientation * T_ES.translation();
  }

  Eigen::Matrix<double, 6, 6> axes(const Eigen::Quaterniond& orientation) const {
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.topLeftCorner<3, 3>() = orientation.toRotationMatrix() * T_ES.linear();
    Q.bottomRightCorner<3, 3>() = Q.topLeftCorner<3, 3>();
    return Q;  // Q_BS: local S vectors -> robot base vectors, at the same point.
  }

  static Eigen::Matrix3d skew(const Eigen::Vector3d& r) {
    Eigen::Matrix3d result;
    result << 0, -r.z(), r.y(), r.z(), 0, -r.x(), -r.y(), r.x(), 0;
    return result;
  }

  template<class Derived>
  static void shiftJacobian(Eigen::MatrixBase<Derived>& J, const Eigen::Vector3d& r) {
    J.template topRows<3>() -= skew(r) * J.template bottomRows<3>();
  }
};
}  // namespace compliant_controllers
