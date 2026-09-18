# compliant_controllers

Robot-agnostic compliant controllers for ROS 2 using `ros2_control`.

## Architecture

The architecture separates robot dynamics, control data, the control law, and
ROS 2 integration:

- `RobotModel`, provided by `ros2_control_robot_dynamics`, computes end-effector
  kinematics, the Jacobian `J(q)`, mass matrix `M(q)`, Coriolis/centrifugal torques,
  and gravity torques `g(q)` from the robot description and joint state.
- `ControlCommand` carries desired Cartesian or joint targets, stiffness,
  damping, and feedforward inputs in a ROS-independent data structure.
- `ControllerState`, defined in `ControlStates.hpp`, carries current joint
  positions, velocities, measured torques, and end-effector pose.
- `control::AbstractController` defines
  `step(command, current_state, control_output, dt)`: one control-law evaluation
  that writes the output joint torques. Implementations can access the externally
  owned robot model through `setRobotModel(...)` and build as plain shared libraries.
- `GenericCartesianControllerWrapper` and `GenericJointControllerWrapper` are
  the `ros2_control` plugins. They handle lifecycle and ROS communication, read
  hardware state, update the model and control data, call `step(...)`, and apply
  compensation and torque limits before writing effort commands to the hardware.
  Select the implementation library with `impl_library`.

The package includes reference Cartesian and joint impedance implementations.
Robot description loading and Pinocchio-backed dynamics are provided by
`ros2_control_robot_dynamics`. 

This separation keeps controller logic independent of ROS 2 and robot-specific bringup, and supports implementations written in C++ or generated through the [Simulink pipeline](https://github.com/smihael/compliant_controllers_simulink_pipeline).

See [Architecture.md](Architecture.md) for more details.

## Build

Place this repository, `compliant_controllers_msgs`, and
`ros2_control_robot_dynamics` in your ROS 2 workspace and install their dependencies.
From the workspace root, source your ROS distribution and build the package with
its workspace dependencies:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-up-to compliant_controllers --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

For the shared Docker setup, run this from the parent workspace containing its
`Dockerfile`:

```bash
docker build -t compliant-controllers:base .
```

## Robot demos

Robot-specific launch files, controller configurations, Docker Compose services,
and Gazebo/MuJoCo setup live in
[compliant_controllers_demos](https://github.com/smihael/compliant_controllers_demos).
Follow the relevant demo README for hardware or simulator startup. With a controller manager running, inspect its controllers and hardware interfaces. Replace `/controller_manager` with the namespaced path when applicable:

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

## Cartesian command example

`test_cartesian_command.py` sends a Cartesian offset from the current end-effector
pose. Run it with the same ROS domain as the controller and matching namespace and
frames. For the Franka Gazebo demo, this command requests a 5 mm X offset:

```bash
export ROS_DOMAIN_ID=1
ros2 run compliant_controllers test_cartesian_command.py \
  --ros-args -r __ns:=/ \
  -p controller_name:=cartesian_impedance_controller \
  -p base_frame:=fr3_link0 -p ee_frame:=fr3_link8 \
  -p dx:=0.005 -p dy:=0.0 -p dz:=0.0 \
  -p k_lin:=300.0 -p k_rot:=20.0 \
```



## Reference

If you are using this package, consider citing:

```bib
@misc{simonic2026plugplaycomply,
      title={Plug, Play, and Comply: A Modular Framework for Online Variable Impedance with Arbitrarily Oriented Compliance Axes},
      author={Mihael Simoni\v{c} and Xiaocong Li},
      year={2026},
      eprint={2607.22483},
      archivePrefix={arXiv},
      url={https://arxiv.org/abs/2607.22483},
}
```

