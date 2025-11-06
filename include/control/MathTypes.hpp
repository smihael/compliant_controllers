#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace control {

using Vec3       = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using Matrix3    = Eigen::Matrix3d;
using VectorX    = Eigen::VectorXd;

struct Pose {
    Vec3 position = Vec3::Zero();
    Quaternion orientation = Quaternion::Identity();
};

struct Twist {
    Vec3 linear  = Vec3::Zero();
    Vec3 angular = Vec3::Zero();
};

struct Accel {
    Vec3 linear  = Vec3::Zero();
    Vec3 angular = Vec3::Zero();
};

struct Wrench {
    Vec3 force  = Vec3::Zero();
    Vec3 torque = Vec3::Zero();
};

} // namespace control
