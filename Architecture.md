# Architecture

`compliant_controllers` provides ROS 2 `ros2_control` controller plugins that wrap dynamically loaded, ROS-independent controller implementations. It provides two exported wrapper plugins:

- `GenericCartesianControllerWrapper`
- `GenericJointControllerWrapper`

Both wrappers handle ROS 2 lifecycle integration, hardware interface claiming, state reads, model updates, dynamic implementation loading, torque sanitization, and effort command writes. The implementation libraries remain plain shared libraries behind the `control::AbstractController` interface.

The refactor also moved robot model ownership out of this package. URDF loading and Pinocchio-backed model operations now come from the shared `ros2_control_robot_dynamics` package:

- `RobotDescriptionLoader`
- `RobotModel`

This package now focuses on controller wrappers, built-in implementation libraries, torque-domain compensation, and diagnostics. Robot-specific launch/config examples live in `compliant_controllers_demos`.

## Design Goals

The architecture is guided by these goals:

- Keep `update()` deterministic and compact.
- Parse and validate parameters during lifecycle configuration, not in the realtime loop.
- Keep controller implementations independent of ROS 2 boilerplate.
- Allow Cartesian and joint-space implementations to be selected dynamically through `impl_library`.
- Share robot model and robot description handling through `ros2_control_robot_dynamics`.
- Keep robot-specific launch/config dependencies outside the core controller package.
- Preserve compatibility with multiple robots by requiring explicit joint names in controller configuration.

## High-Level Architecture

The current runtime can be viewed as:

```text
ros2_control ControllerManager
        |
        +--> GenericCartesianControllerWrapper
        |        |
        |        +-- RobotDescriptionLoader   (ros2_control_robot_dynamics)
        |        +-- RobotModel              (ros2_control_robot_dynamics)
        |        +-- dynamically loaded AbstractController implementation
        |        +-- Gravity compensation
        |        +-- FrictionCompensation
        |        +-- End-effector load compensation
        |        +-- AsyncDiagnosticLogger
        |
        +--> GenericJointControllerWrapper
                 |
                 +-- RobotDescriptionLoader   (ros2_control_robot_dynamics)
                 +-- RobotModel              (ros2_control_robot_dynamics)
                 +-- dynamically loaded AbstractController implementation
                 +-- ...
```

The wrappers own lifecycle and hardware interface interaction. `ros2_control_robot_dynamics` owns URDF/model concerns. Built-in implementations and compensation modules stay in this package.

## Main Components

### Wrapper Plugins

#### `GenericCartesianControllerWrapper`

The Cartesian wrapper is an exported `controller_interface::ControllerInterface` plugin for Cartesian commands.

Responsibilities:

- Declare command and state interface requirements.
- Read joint position, velocity, and effort from `state_interfaces_`.
- Subscribe to `compliant_controllers_msgs/msg/CartesianCommand`.
- Maintain a realtime Cartesian command buffer.
- Load and initialize the robot model through `ros2_control_robot_dynamics`.
- Dynamically load a controller implementation from `impl_library`.
- Seed the initial Cartesian command from the current end-effector pose on activation.
- Apply optional gravity, friction, and end-effector load compensation.
- Record optional asynchronous diagnostics.
- Sanitize and write torque output.
- Emit lifecycle and first-update diagnostics.

#### `GenericJointControllerWrapper`

The joint wrapper is an exported `controller_interface::ControllerInterface` plugin for joint-space commands.

Responsibilities:

- Declare command and state interface requirements.
- Read joint position, velocity, and effort from `state_interfaces_`.
- Subscribe to `compliant_controllers_msgs/msg/JointCommand`.
- Maintain a realtime joint command buffer.
- Load and initialize the robot model through `ros2_control_robot_dynamics`.
- Dynamically load a controller implementation from `impl_library`.
- Seed initial joint targets on activation.
- Apply optional friction compensation.
- Sanitize and write torque output.
- Emit lifecycle and first-update diagnostics.

### `ros2_control_robot_dynamics`

The model boundary is external to this package.

`RobotDescriptionLoader` handles URDF retrieval from:

- local `robot_description` parameter
- remote `robot_description_node` and `robot_description_param`

`RobotModel` owns the URDF-backed kinematic and dynamic model abstraction. Wrappers and implementations use it to:

- initialize from URDF and frame configuration
- update model state from current joint positions
- query end-effector pose
- query Jacobian or Jacobian pseudoinverse
- query gravity, Coriolis, and inertia terms where needed

This package should continue to depend on the model through the high-level API instead of accessing Pinocchio directly in wrappers.

### Dynamic Controller Implementations

Controller implementations inherit from `control::AbstractController` and are loaded from shared libraries using the factory symbols expected by the wrappers.

Current built-in implementation libraries:

- `libcartesian_impedance_impl.so`
- `libjoint_impedance_impl.so`

The wrapper forwards state, command, model pointer, and `dt` into:

```cpp
impl_->step(command, current_state, tau_out, dt);
```

The implementation writes joint torque commands into the provided output vector. The wrapper remains responsible for final compensation, sanitization, and hardware writes.

### `CartesianImpedanceImpl`

`CartesianImpedanceImpl` is a ROS-independent Cartesian impedance implementation. It uses `RobotModel` for pose, Jacobian, and pseudoinverse queries, and computes a torque command from Cartesian pose error, stiffness/damping gains, desired wrench, and nullspace behavior.

### `JointImpedanceImpl`

`JointImpedanceImpl` is a ROS-independent joint-space impedance implementation. It supports filtered joint targets, stiffness/damping parameters, torque-rate saturation, and a power-enable guard before producing full commanded torques.

### `AsyncDiagnosticLogger`

`AsyncDiagnosticLogger` records selected runtime samples without writing files directly from the control loop.

It accepts:

- joint position and velocity
- measured and commanded torque
- gravity and Coriolis vectors
- inertia matrix
- diagnostic log filter tag

Samples are queued from the wrapper and written by a background thread. This keeps normal update-time work bounded while still allowing short diagnostic captures.

## Lifecycle Flow

### `on_init()`

The wrappers:

- create command subscriptions
- create diagnostic log filter tag subscriptions where supported
- optionally enable debug behavior when configured
- log wrapper initialization

### `on_configure()`

The wrappers:

- read wrapper-owned parameters such as `arm_id`, joint names, and `impl_library`
- configure compensation modules
- configure preallocated buffers
- load the robot description through `RobotDescriptionLoader`
- initialize `RobotModel`
- dynamically load the selected controller implementation
- inject the robot model pointer into the implementation
- forward implementation-specific parameters where supported
- log a configuration summary

Parameter lookup and validation should happen here, not in `update()`.

### `on_activate()`

The wrappers:

- read initial controller state
- update the robot model
- seed initial command targets from current state
- run or validate an initial implementation step where needed
- refuse activation if the implementation cannot produce a valid initial output
- log activation details
- reset one-shot first-update logging

### `update()`

The Cartesian update path is:

```text
readControllerState()
robot_model_.update(q)
robot_model_.getPose(...)
latest command = rt_cartesian_cmd_buffer_.readFromRT()
impl_->step(...)
addGravityCompensation()
addEndEffectorLoadCompensation()
addFrictionCompensation(dt)
sanitizeTorqueOutput()
diagnostic_logger_.record(...)
write torques
logFirstUpdateSummary()
```

The joint update path is:

```text
readControllerState()
robot_model_.update(q)
latest command = rt_joint_cmd_buffer_.readFromRT()
impl_->step(...)
friction_compensation_.add(...)
sanitizeTorqueOutput()
writeTorqueOutput()
logFirstUpdateSummary()
```

### Realtime Considerations

The intended realtime policy is:
- No ROS parameter reads in `update()`.
- No dynamic model selection in `update()`.
- Compensation vectors are validated and allocated during configuration.
- File writes are delegated to `AsyncDiagnosticLogger` instead of being performed directly in the control loop.
- `update()` only reads hardware state, runs model/controller math, adds preconfigured compensation, and writes torques.

## Package Boundaries

The core package contains:

- wrapper plugins
- built-in implementation libraries
- compensation modules
- diagnostic logging
- generic wrapper launch includes
- test/helper scripts that directly exercise this package

The demos package contains platform-specific launch and configuration files.

## Timing Tests

The optional `COMPLIANT_CONTROLLERS_BUILD_TIMING_TESTS` CMake option builds standalone timing executables:

- `joint_update_timing_test`
- `cartesian_update_timing_test`

These tests measure implementation and model-update timing without communication. They can be used to assess whether the update step provides sufficient computational headroom for the desired control-loop frequency.