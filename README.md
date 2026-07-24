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

ros2 launch franka_bringup example.launch.py controller_name:=cartesian_impedance_controller

ros2 launch compliant_controllers gazebo.launch.py show_gazebo_gui:=true

ros2 run compliant_controllers test_cartesian_command.py --ros-args -p dz:=-0.05

## Parameters
- `arm_id` (string): prefix for joint names (default: `panda`).
- `k_p`, `k_d` (double): uniform gains applied to all joints.

## Extending
Add additional controllers following the pattern: implement lifecycle hooks, define interfaces, export in `compliant_controllers.xml` and add to library target in `CMakeLists.txt`.
