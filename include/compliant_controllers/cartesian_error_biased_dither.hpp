#pragma once

#include <Eigen/Eigen>

#include <control/ControlCommand.hpp>
#include <control/ControlStates.hpp>

namespace compliant_controllers {

// Real-time Cartesian error-biased dither. Parameters intentionally live here
// while the method is being evaluated; no ROS parameters are consumed.
class CartesianErrorBiasedDither {
public:
  static constexpr double kFrequencyHz = 50.0;
  static constexpr double kAlpha = 50.0;
  static constexpr double kBeta = 0.8;
  static constexpr double kBaseAmplitudeNm = 1.5;

  void reset();

  bool compute(const control::ControlCommand& command,
               const control::ControllerState& state,
               double dt,
               Eigen::Ref<Eigen::Matrix<double, 6, 1>> wrench_out);

private:
  double phase_rad_{0.0};
};

}  // namespace compliant_controllers
