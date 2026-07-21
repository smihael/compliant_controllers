#include <compliant_controllers/cartesian_error_biased_dither.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace compliant_controllers {

namespace {

Eigen::Vector3d rotationCorrectionWorld(const Eigen::Quaterniond& current,
                                        const Eigen::Quaterniond& desired) {
  Eigen::Quaterniond error = current.normalized().inverse() * desired.normalized();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }

  const double w = std::clamp(error.w(), -1.0, 1.0);
  const double angle = 2.0 * std::acos(w);
  const double sin_half_angle = std::sqrt(std::max(0.0, 1.0 - w * w));
  if (angle < 1e-9 || sin_half_angle < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d axis_local(
      error.x() / sin_half_angle,
      error.y() / sin_half_angle,
      error.z() / sin_half_angle);
  return current.normalized().toRotationMatrix() * (angle * axis_local);
}

}  // namespace

void CartesianErrorBiasedDither::reset() {
  phase_rad_ = 0.0;
}

bool CartesianErrorBiasedDither::compute(
    const control::ControlCommand& command,
    const control::ControllerState& state,
    double dt,
  Eigen::Ref<Eigen::Matrix<double, 6, 1>> wrench_out) {
  wrench_out.setZero();
  if (!std::isfinite(dt) || dt <= 0.0 ||
      !command.orientation.coeffs().array().isFinite().all() ||
      !state.orientation.coeffs().array().isFinite().all() ||
      command.orientation.norm() < 1e-9 || state.orientation.norm() < 1e-9) {
    return false;
  }

  // The reported experiment excites rotation about the world-aligned y axis.
  // Positive error means that a positive y torque moves toward the target.
  const double error_y = rotationCorrectionWorld(
      state.orientation, command.orientation).y();
  const double bias = kBeta * std::tanh(kAlpha * error_y);
  const double torque_y = kBaseAmplitudeNm * (std::sin(phase_rad_) + bias);
  if (!std::isfinite(torque_y)) {
    return false;
  }

  wrench_out(4) = torque_y;
  phase_rad_ = std::fmod(
      phase_rad_ + 2.0 * std::numbers::pi * kFrequencyHz * dt,
      2.0 * std::numbers::pi);
  return true;
}

}  // namespace compliant_controllers
