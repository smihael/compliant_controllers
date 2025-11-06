#pragma once
#include <Eigen/Core>

namespace compliant_controllers {

struct RobotState {
    Eigen::VectorXd q;   // joint positions
    Eigen::VectorXd dq;  // joint velocities
    Eigen::VectorXd tau; // measured joint torques
};

} // namespace compliant_controllers
