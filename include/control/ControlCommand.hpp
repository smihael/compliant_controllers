#pragma once
#include "ControlStates.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace control {

struct ControlCommand {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Matrix<double,6,1> velocity{Eigen::Matrix<double,6,1>::Zero()};
    Eigen::Matrix<double,6,1> wrench{Eigen::Matrix<double,6,1>::Zero()};
    Eigen::Matrix<double,6,6> stiffness{Eigen::Matrix<double,6,6>::Zero()};
    Eigen::Matrix<double,6,6> damping{Eigen::Matrix<double,6,6>::Zero()};
    Eigen::VectorXd q_ns_des; 
    Eigen::VectorXd k_ns;    
    Eigen::VectorXd d_ns;     
    Eigen::VectorXd tau_ff;  

    ControlCommand() = default;
    explicit ControlCommand(size_t n)
            : q_ns_des(Eigen::VectorXd::Zero(static_cast<long>(n))),
              k_ns(Eigen::VectorXd::Zero(static_cast<long>(n))),
              d_ns(Eigen::VectorXd::Zero(static_cast<long>(n))),
              tau_ff(Eigen::VectorXd::Zero(static_cast<long>(n))) {}
};

} // namespace control
