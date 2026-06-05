#pragma once

#include <control/AbstractController.hpp>

#include <Eigen/Dense>

#include <array>
#include <string>
#include <variant>

#include "friction_compensation.h"

namespace compliant_controllers {

class FrictionCompensationImplV2 : public control::AbstractController {
public:
  static constexpr const char* kName = "FrictionCompensationImplV2";

  explicit FrictionCompensationImplV2(int num_joints);
  ~FrictionCompensationImplV2() override = default;

  bool step(const control::ControlCommand& command,
            const control::ControllerState& current_state,
            Eigen::Ref<Eigen::VectorXd> control_output,
            double dt) override;

    void setParameter(const std::string& name, const ParameterValue& value) override;

private:
  int num_joints_;
    bool initialized_{false};
  double friction_compensation_{0.0};

    friction_compensation::FrictionModelParams friction_model_params_{};
    std::array<friction_compensation::SingleJointFrictionCompensator, 7>
      friction_compensators_{
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[0],
          friction_compensation::kDataDefault.joints[0]),
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[1],
          friction_compensation::kDataDefault.joints[1]),
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[2],
          friction_compensation::kDataDefault.joints[2]),
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[3],
          friction_compensation::kDataDefault.joints[3]),
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[4],
          friction_compensation::kDataDefault.joints[4]),
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[5],
          friction_compensation::kDataDefault.joints[5]),
        friction_compensation::SingleJointFrictionCompensator(
          friction_model_params_.joints[6],
          friction_compensation::kDataDefault.joints[6]),
      };
};

}  // namespace compliant_controllers
