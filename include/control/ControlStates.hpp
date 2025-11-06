#pragma once
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace control {
struct ControllerState {
        Eigen::VectorXd q;   // joint positions
        Eigen::VectorXd dq;  // joint velocities
        Eigen::VectorXd tau; // measured joint torques
        
        Eigen::Vector3d position; // end-effector position
        Eigen::Quaterniond orientation; // end-effector orientation

        ControllerState() = default;
        ControllerState(size_t n)
            : q(Eigen::VectorXd::Zero(n)), dq(Eigen::VectorXd::Zero(n)), tau(Eigen::VectorXd::Zero(n)) {}
        size_t size() const { return static_cast<size_t>(q.size()); }
};
}
