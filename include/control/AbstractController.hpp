#pragma once
#include "ControlCommand.hpp"
#include "ControlStates.hpp"
#include <string>
#include <type_traits>

namespace control {

class AbstractController {
public:
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

};

} // namespace control

// Export factory + name symbols (extern "C") for a controller implementation Type.
// The Type must define: static constexpr const char* kName;
#define FACTORY_EXPORT_CONTROLLER(Type) \
    extern "C" { \
        static_assert(std::is_base_of<control::AbstractController, Type>::value, "Type must derive from control::AbstractController"); \
        control::AbstractController* create_controller() { return new Type(); } \
        void destroy_controller(control::AbstractController* c) { delete c; } \
        const char* controller_name() { return Type::kName; } \
    }
