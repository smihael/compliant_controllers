These headers define a small framework-agnostic control interface used by compliant_controllers:

- MathTypes.hpp: Eigen-based math aliases and small structs.
- ControlStates.hpp: State structs for Cartesian and Joint control.
- ControlCommand.hpp: A union-like command container with mode.
- AbstractController.hpp: Pure abstract interface for control implementations.

They are designed to be used by ROS wrappers, Simulink shims, or plain C++ apps.
