#pragma once

#include <control/AbstractController.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace compliant_controllers {

class FrictionEstimationImpl : public control::AbstractController {
  // If >0, only this joint is tested (Simulink style); otherwise, all joints are tested sequentially (ROS default)
  int test_joint_index_{-1};
public:
  static constexpr const char* kName = "FrictionEstimationImpl";

  explicit FrictionEstimationImpl(int num_joints);
  ~FrictionEstimationImpl() override;

  bool step(const control::ControlCommand& command,
            const control::ControllerState& current_state,
            Eigen::Ref<Eigen::VectorXd> control_output,
            double dt) override;
  void setParameter(const std::string& name, const control::AbstractController::ParameterValue& value) override;

private:
  enum class Phase {
    MoveToQInit,
    SettleAtQInit,
    ExciteJoint,
    ReturnToQInit,
    Done,
  };

  struct Sample {
    double t;
    int active_joint;
    bool excitation;
    double q;
    double dq;
    double tau_measured;
    double tau_commanded;
    double q_des;
    double dq_des;
    double sine_tau_ff;
  };

  void readForwardedParameters();

  bool initializeRun(const control::ControllerState& state);
  void writeCsvIfNeeded();
  void estimateAndWriteMatricesIfNeeded();
  bool updateRobotConfigFromModel2(const Eigen::MatrixXd& model2);
  void writeDoneMarkerIfNeeded();
  void requestShutdownIfNeeded();
  double clamp(double x, double low, double high) const;
  bool isAtTarget(const control::ControllerState& state, const Eigen::Ref<const Eigen::VectorXd>& q_target) const;
  void computeHoldTorques(const control::ControllerState& state,
                          const Eigen::Ref<const Eigen::VectorXd>& q_target,
                          Eigen::Ref<Eigen::VectorXd> tau_out,
                          const Eigen::VectorXd* dq_target = nullptr) const;
  void computeMoveTorques(const control::ControllerState& state,
                          const Eigen::Ref<const Eigen::VectorXd>& q_target,
                          double dt,
                          Eigen::Ref<Eigen::VectorXd> tau_out);
  bool checkMotionWatchdog(const control::ControllerState& state,
                           const Eigen::Ref<const Eigen::VectorXd>& tau_cmd,
                           const Eigen::Ref<const Eigen::VectorXd>& dq_des,
                           double dt);
  double ramp01(double t, double slope, double start) const;
  double motionSignal(double t, int joint_one_based) const;
  std::string buildDefaultOutputPath() const;
  const char* phaseName(Phase phase) const;
  void setPhase(Phase next_phase, const char* reason = nullptr);

  int num_joints_;
  bool initialized_{false};
  bool init_logged_{false};
  bool finished_{false};
  bool csv_written_{false};
  bool matrices_written_{false};
  bool config_update_written_{false};
  bool done_marker_written_{false};
  bool shutdown_requested_{false};

  double sim_time_{0.0};
  double stage_time_{0.0};
  double phase_enter_time_s_{0.0};
  double last_move_progress_log_s_{-1.0};
  double last_phase_heartbeat_log_s_{-1.0};
  int active_joint_{0};
  Phase phase_{Phase::MoveToQInit};

  double q_init_tolerance_rad_{0.07};
  double settle_time_s_{0.5};
  double return_settle_time_s_{0.5};
  double measurement_time_s_{8.0};
  double amplitude_rad_{1.0};
  double excitation_frequency_hz_{1.0};
  double excitation_tau_ff_nm_{0.0};
  double hold_kp_{50.0};
  double hold_kd_{5.0};
  double tau_abs_limit_nm_{20.0};
  double move_max_vel_rad_s_{0.10};
  double move_tau_limit_nm_{5.0};
  double move_kp_scale_{0.20};
  double move_kd_scale_{0.35};
  double move_progress_log_period_s_{2.0};
  double phase_log_period_s_{2.0};
  double watchdog_dq_min_rad_s_{0.005};
  double watchdog_q_delta_min_rad_{1e-5};
  double watchdog_tau_cmd_min_nm_{0.5};
  double watchdog_stall_timeout_s_{0.8};
  int start_joint_index_{1};
  int end_joint_index_{-1};
  std::string output_csv_path_{"/tmp/friction_estimation_measurements.csv"};
  std::string output_model1_csv_path_{"/tmp/model1_estimated_friction.csv"};
  std::string output_model2_csv_path_{"/tmp/model2_estimated_friction.csv"};
  std::string output_matlab_path_{"/tmp/friction_estimated_friction.m"};
  std::string output_dir_{"/tmp"};
  double model2_hysteresis_{0.05};
  bool update_robot_config_{true};
  bool shutdown_when_done_{true};
  std::string robot_profile_{};
  std::string robot_name_for_profile_{};
  std::string updated_robot_config_output_{"/tmp/robot_config.updated.yaml"};
  std::string done_marker_file_{"/tmp/friction_estimation.done"};
  std::chrono::system_clock::time_point run_started_{};
  bool parameters_applied_{false};
  std::map<std::string, control::AbstractController::ParameterValue> forwarded_params_;

  Eigen::VectorXd q0_;
  Eigen::VectorXd q_init_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd k_gains_;
  Eigen::VectorXd d_gains_;
  Eigen::VectorXd amplitudes_rad_;
  Eigen::VectorXd durations_s_;
  Eigen::VectorXd move_max_vels_rad_s_;
  Eigen::VectorXd q_move_ref_;
  Eigen::VectorXd prev_q_;
  double stall_elapsed_s_{0.0};
  std::vector<Sample> samples_;
};

}  // namespace compliant_controllers
