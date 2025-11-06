#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace compliant_controllers {

struct ControlCommand {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Matrix<double,6,1> velocity{Eigen::Matrix<double,6,1>::Zero()};
    Eigen::Matrix<double,6,1> wrench{Eigen::Matrix<double,6,1>::Zero()};
    Eigen::Matrix<double,6,6> stiffness{Eigen::Matrix<double,6,6>::Zero()};
    Eigen::Matrix<double,6,6> damping{Eigen::Matrix<double,6,6>::Zero()};
    Eigen::VectorXd q_ns_des;   // desired nullspace joint positions
    Eigen::VectorXd k_ns;       // nullspace stiffness per joint
    Eigen::VectorXd d_ns;       // nullspace damping per joint
    Eigen::VectorXd tau_ff;     // joint torque feedforward
    bool has_value{false};
};

} // namespace compliant_controllers
