# compliant_controllers

Example compliant (impedance-style) controllers for Franka robots using ros2_control.

Currently implemented:
- `CartesianImpedanceController`: placeholder Cartesian impedance controller (currently joint-space PD), subscribing to a `sensor_msgs/JointState` command topic to set target joint positions.

## Build
```
colcon build --packages-select compliant_controllers
source install/setup.bash
```

## Usage
1. Ensure your Franka hardware (or simulation) is running with a ros2_control `controller_manager`.
2. Load parameters (if not already via main bringup):
```
ros2 param load /controller_manager compliant_controllers/config/controllers.yaml
```
3. Spawn controller:
```
ros2 run controller_manager spawner cartesian_impedance_controller
```
4. Send a command:
```
ros2 topic pub /cartesian_impedance_controller/command sensor_msgs/JointState '{name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0, -0.2, 0, -1.5, 0, 1.0, 0.5]}' --once
```

## Parameters
- `arm_id` (string): prefix for joint names (default: `panda`).
- `k_p`, `k_d` (double): uniform gains applied to all joints.

## Extending
Add additional controllers following the pattern: implement lifecycle hooks, define interfaces, export in `compliant_controllers.xml` and add to library target in `CMakeLists.txt`.
