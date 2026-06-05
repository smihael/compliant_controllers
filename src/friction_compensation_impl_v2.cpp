#include <compliant_controllers/friction_compensation_impl_v2.hpp>

#include <cmath>
#include <iostream>
#include <variant>

namespace {
bool allFinite(const Eigen::VectorXd& v) {
  return v.array().isFinite().all();
}

double parameterToDouble(const control::AbstractController::ParameterValue& value) {
  return std::visit(
      [](const auto& raw) -> double {
        using T = std::decay_t<decltype(raw)>;
        if constexpr (std::is_same_v<T, double>) {
          return raw;
        } else if constexpr (std::is_same_v<T, int64_t>) {
          return static_cast<double>(raw);
        } else if constexpr (std::is_same_v<T, bool>) {
          return raw ? 1.0 : 0.0;
        } else {
          return 0.0;
        }
      },
      value);
}

bool parameterToBool(const control::AbstractController::ParameterValue& value) {
  return std::visit(
      [](const auto& raw) -> bool {
        using T = std::decay_t<decltype(raw)>;
        if constexpr (std::is_same_v<T, bool>) {
          return raw;
        } else if constexpr (std::is_same_v<T, double>) {
          return raw != 0.0;
        } else if constexpr (std::is_same_v<T, int64_t>) {
          return raw != 0;
        } else {
          return false;
        }
      },
      value);
}
}  // namespace

namespace compliant_controllers {

FrictionCompensationImplV2::FrictionCompensationImplV2(int num_joints)
    : num_joints_(num_joints) {
  for (int i = 0; i < 7; ++i) {
    friction_model_params_.joints[i].use_gating = true;
  }

  std::cout << "\033[32mUsing " << kName << " compiled at " << __DATE__ << ", "
            << __TIME__ << "\033[0m" << std::endl;
}

bool FrictionCompensationImplV2::step(const control::ControlCommand& /*command*/,
                                      const control::ControllerState& current_state,
                                      Eigen::Ref<Eigen::VectorXd> control_output,
                                      double dt) {
  if (!initialized_) {
    constexpr double kInitDt = 1e-3;
    for (int i = 0; i < 7; ++i) {
      friction_compensators_[i].Reset();
      friction_compensators_[i].Initialize(current_state.q(i), current_state.dq(i), kInitDt);
    }
    initialized_ = true;
  }

  control_output.setZero();
  for (int i = 0; i < 7; ++i) {
    const double tau = friction_compensators_[i].Step(dt, current_state.q(i), current_state.dq(i));
    control_output(i) = friction_compensation_ * tau;
  }

  if (!allFinite(control_output)) {
    std::cerr << "[FrictionCompensationImplV2::step] non-finite friction output." << std::endl;
    control_output.setZero();
    return false;
  }

  return true;
}

void FrictionCompensationImplV2::setParameter(const std::string& name,
                                              const ParameterValue& value) {
  if (name == "plugin_params_file") {
    if (const auto* path = std::get_if<std::string>(&value); path != nullptr && !path->empty()) {
      if (!loadParametersFromFile(*path)) {
        std::cerr << "[FrictionCompensationImplV2::setParameter] failed to load plugin params file: "
                  << *path << std::endl;
      }
    }
    return;
  }

  if (name == "friction_compensation" || name == "friction_compensation.scale") {
    friction_compensation_ = parameterToDouble(value);
    return;
  }

  if (name == "use_gating") {
    const bool use_gating = parameterToBool(value);
    for (int i = 0; i < 7; ++i) {
      friction_model_params_.joints[i].use_gating = use_gating;
    }
    return;
  }
}

}  // namespace compliant_controllers

FACTORY_EXPORT_CONTROLLER(compliant_controllers::FrictionCompensationImplV2)
