#pragma once
#include "ControlCommand.hpp"
#include "ControlStates.hpp"
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace control {

class AbstractController {
public:
    using ParameterValue = std::variant<
        bool,
        int64_t,
        double,
        std::string,
        std::vector<int64_t>,
        std::vector<double>,
        std::vector<std::string>>;

    virtual ~AbstractController() = default;

    /**
     * @brief Perform one control step.
     * @param command Input control command
     * @param current_state Current robot state.
     * @param control_output Output torques/forces.
     * @param dt Time step in seconds.
     */
    virtual bool step(const ControlCommand& command,
                      const ControllerState& current_state,
                      Eigen::Ref<Eigen::VectorXd> control_output,
                      double dt) = 0;

    // Optional injection of robot model owned externally
    virtual void setRobotModel(void* /*model_ptr*/) {}

    // Optional typed parameter forwarding from wrapper to implementation.
    virtual void setParameter(const std::string& /*name*/, const ParameterValue& /*value*/) {}

    // Legacy compatibility stub; concrete controllers can implement their own parsers.
    bool loadParametersFromFile(const std::string& /*path*/) { return false; }

};

} // namespace control

// Export factory + name symbols (extern "C") for a controller implementation Type.
// The Type must define: static constexpr const char* kName;
#define FACTORY_EXPORT_CONTROLLER(Type) \
    extern "C" { \
        static_assert(std::is_base_of<control::AbstractController, Type>::value, "Type must derive from control::AbstractController"); \
        control::AbstractController* create_controller(int num_joints) { return new Type(num_joints); } \
        void destroy_controller(control::AbstractController* c) { delete c; } \
        const char* controller_name() { return Type::kName; } \
    }
