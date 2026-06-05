#pragma once

#include <Eigen/Eigen>

namespace compliant_controllers {

struct MotionGeneratorState {
  double time_s{0.0};
  double phase_time_s{0.0};
  int phase{0};
  int active_joint{-1};
  Eigen::VectorXd q;
  Eigen::VectorXd dq;
};

struct MotionGeneratorTarget {
  Eigen::VectorXd q;
  Eigen::VectorXd dq;
  // Runtime phase/mode tag intended for diagnostic logging.
  int mode{0};
};

class MotionGenerator {
public:
  virtual ~MotionGenerator() = default;

  virtual bool initialize(int num_joints,
                          const Eigen::VectorXd& q_initial) = 0;

  virtual bool target(const MotionGeneratorState& state,
                      MotionGeneratorTarget& target_out) = 0;
};

}  // namespace compliant_controllers
