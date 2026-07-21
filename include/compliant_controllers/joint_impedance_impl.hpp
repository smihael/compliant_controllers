#pragma once

#include <Eigen/Dense>

#include <control/AbstractController.hpp>
#include <ros2_control_robot_dynamics/robot_model.hpp>

namespace compliant_controllers {

class JointImpedanceImpl : public control::AbstractController {
public:
  static constexpr const char* kName = "JointImpedanceImpl";

  explicit JointImpedanceImpl(int num_joints);
  ~JointImpedanceImpl() override = default;

  bool step(const control::ControlCommand& command,
            const control::ControllerState& current_state,
            Eigen::Ref<Eigen::VectorXd> control_output,
            double dt) override;

  void setRobotModel(void* model_ptr) override { robot_model_ = static_cast<RobotModel*>(model_ptr); }

private:
  bool validInput(const control::ControlCommand& command,
                  const control::ControllerState& current_state,
                  const Eigen::Ref<const Eigen::VectorXd>& control_output) const;
  void initializeTargets(const control::ControlCommand& command);
  void filterTargets(const control::ControlCommand& command);
  void saturateTorqueRate(const Eigen::Ref<const Eigen::VectorXd>& tau_desired,
                          Eigen::Ref<Eigen::VectorXd> tau_saturated);

  int num_joints_{0};
  RobotModel* robot_model_{nullptr};

  Eigen::VectorXd coriolis_;
  Eigen::VectorXd tau_calculated_;
  Eigen::VectorXd tau_saturated_;
  Eigen::VectorXd tau_last_commanded_;

  Eigen::VectorXd q_d_;
  Eigen::VectorXd dq_d_;
  Eigen::VectorXd k_gains_;
  Eigen::VectorXd d_gains_;
  Eigen::VectorXd q_d_target_;
  Eigen::VectorXd dq_d_target_;
  Eigen::VectorXd k_gains_target_;
  Eigen::VectorXd d_gains_target_;
  Eigen::VectorXd feed_forward_q_target_;

  double filter_alpha_{0.99};
  double max_tau_delta_{1.0};
  bool initialized_{false};
};

}  // namespace compliant_controllers
