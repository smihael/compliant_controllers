#pragma once

#include <control/AbstractController.hpp>
#include <Eigen/Dense>
#include <string>
#include <memory>

#include <compliant_controllers/robot_model.hpp>

namespace compliant_controllers {

// Framework-agnostic Cartesian impedance implementation
class CartesianImpedanceImpl : public control::AbstractController {
public:

  static constexpr const char* kName = "CartesianImpedanceImpl";

  explicit CartesianImpedanceImpl(int num_joints);
  ~CartesianImpedanceImpl() override = default;

  bool step(const control::ControlCommand& command,
            const control::ControllerState& current_state,
            Eigen::Ref<Eigen::VectorXd> control_output,
            double dt) override;

  void setRobotModel(void* model_ptr) override { robot_model_ = static_cast<RobotModel*>(model_ptr); }


private:
  // Helpers
  Eigen::Vector3d q_log(const Eigen::Quaterniond &q) const;

  int num_joints_;

  // Robot model and buffers
  RobotModel* robot_model_{nullptr};

  Eigen::Matrix<double,6,Eigen::Dynamic> J_;          // 6 x n Jacobian
  Eigen::Matrix<double,Eigen::Dynamic,6> J_pinv_;     // n x 6 pseudo-inverse J^+ provided by RobotModel
  Eigen::MatrixXd I_n_;                               // n x n identity for nullspace projection
  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;

  // Desired Cartesian state (accumulated from incoming command)

  Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double,6,1> velocity_d_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> wrench_d_{Eigen::Matrix<double,6,1>::Zero()};

  // Gains
  Eigen::Matrix<double,6,6> cartesian_stiffness_{Eigen::Matrix<double,6,6>::Zero()};
  Eigen::Matrix<double,6,6> cartesian_damping_{Eigen::Matrix<double,6,6>::Zero()};

  // Nullspace
  Eigen::VectorXd q_d_nullspace_;
  Eigen::VectorXd nullspace_stiffness_;
  const double TAU_NULLSPACE_MAX_{5.0};
};

} // namespace compliant_controllers
