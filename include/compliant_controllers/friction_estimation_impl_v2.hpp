#pragma once

#include <control/AbstractController.hpp>

#include <ros2_control_robot_dynamics/robot_model.hpp>

#include <Eigen/Dense>

#include <array>
#include <map>
#include <string>
#include <vector>

namespace compliant_controllers {

class FrictionEstimationImplV2 : public control::AbstractController {
public:
  static constexpr const char* kName = "FrictionEstimationImplV2";

  explicit FrictionEstimationImplV2(int num_joints);
  ~FrictionEstimationImplV2() override;

  bool step(const control::ControlCommand& command,
            const control::ControllerState& current_state,
            Eigen::Ref<Eigen::VectorXd> control_output,
            double dt) override;

  void setRobotModel(void* model_ptr) override { robot_model_ = static_cast<RobotModel*>(model_ptr); }

  void setParameter(const std::string& name, const ParameterValue& value) override;

private:
  enum class Phase {
    MoveToQInit,
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
    double tau_task;
    double tau_commanded;
    double tau_inertia;
    double tau_gravity;
    double tau_coriolis;
    double q_des;
    double dq_des;
  };

  class ModifiedFourierSeriesGenerator {
  public:
    explicit ModifiedFourierSeriesGenerator(unsigned seed = 7U);
    double normalizedSignal(double t, double duration_s, int joint_one_based) const;

  private:
    static constexpr int kHarmonics = 4;
    struct JointGenerator {
      std::array<double, kHarmonics> amp{};
      double norm{1.0};
    };

    static JointGenerator makeJointGenerator(unsigned seed);
    static double computeNorm(const std::array<double, kHarmonics>& amp);

    std::array<JointGenerator, 7> joints_{};
  };

  void readForwardedParameters();
  bool initializeRun(const control::ControllerState& state);
  bool isAtTarget(const control::ControllerState& state,
                  const Eigen::Ref<const Eigen::VectorXd>& q_target) const;
  void computeHoldTorques(const control::ControllerState& state,
                          const Eigen::Ref<const Eigen::VectorXd>& q_target,
                          Eigen::Ref<Eigen::VectorXd> tau_out,
                          const Eigen::VectorXd* dq_target = nullptr) const;
  void computeMoveTorques(const control::ControllerState& state,
                          const Eigen::Ref<const Eigen::VectorXd>& q_target,
                          double dt,
                          Eigen::Ref<Eigen::VectorXd> tau_out,
                          Eigen::Ref<Eigen::VectorXd> dq_des_out);
  void writeCsvIfNeeded();
  void requestShutdownIfNeeded();
  std::string buildDefaultOutputPath() const;

  static const char* phaseName(Phase phase);
  void setPhase(Phase phase, const char* reason = nullptr);
  double clamp(double x, double low, double high) const;

  int num_joints_;
  bool initialized_{false};
  bool finished_{false};
  bool csv_written_{false};
  bool shutdown_requested_{false};
  bool parameters_applied_{false};

  double sim_time_{0.0};
  double phase_time_s_{0.0};

  Phase phase_{Phase::MoveToQInit};
  std::size_t active_joint_list_idx_{0};

  Eigen::VectorXd q_init_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;
  Eigen::VectorXd q_move_ref_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd k_gains_;
  Eigen::VectorXd d_gains_;

  std::vector<int> test_joint_indices_;
  std::vector<Sample> samples_;
  std::map<std::string, ParameterValue> forwarded_params_;

  std::string output_csv_path_{"/tmp/friction_estimation_measurements_v2.csv"};

  double amplitude_scaling_{1.0};
  double measurement_time_s_{15.0};
  double q_init_tolerance_rad_{0.03};
  double settle_time_s_{0.9};
  double return_settle_time_s_{0.9};
  double tau_abs_limit_nm_{20.0};
  double move_tau_limit_nm_{8.0};
  double move_max_vel_rad_s_{0.15};

  RobotModel* robot_model_{nullptr};
  Eigen::VectorXd gravity_torque_;
  Eigen::VectorXd coriolis_torque_;
  Eigen::MatrixXd mass_matrix_;
  Eigen::VectorXd prev_dq_des_;

  ModifiedFourierSeriesGenerator mfs_{};

};

}  // namespace compliant_controllers
