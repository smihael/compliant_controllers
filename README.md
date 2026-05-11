## compliant_controllers

Robot-agnostic compliant controllers for ROS 2 using ros2_control.

## Architecture & features
- ROS-free core abstract classes:
	- `RobotModel`: defines interface for computing J(q), C(q, dq), M(q) and g(q)
	- `ControlCommand`, `ControlStates`: lightweight data containers for commands and feedback.
	- `AbstractController`: defines interface for computing joint torques based on the above data containers.
- Sample robot model implementation based on the Pinocchio library that reads the URDF from ROS parameters
- Sample controller implementation: `CartesianImpedanceController : AbstractController` builds as a plain shared library (.so) without ROS deps
- ROS 2 wrapper: `GenericCartesianControllerWrapper` is a ros2_control plugin that instantiates the chosen `AbstractController`, streams states in real time, calls a `RobotModel` implementation and the controller’s `step(...)`, and writes torques.

Thus, the controller logic is fully separated from ROS and the robot. This keeps controller code simple and readable (no ROS 2 boilerplate). Controller implementations can be written in C++ or e.g., Simulink. See https://github.com/smihael/compliant_controllers_simulink_pipeline

## Build and Source

```bash
colcon build --packages-select compliant_controllers --symlink-install
source install/setup.bash
```

You can also build using:
```bash
docker build \
  --ssh default=$SSH_AUTH_SOCK \
  --network=host \
  -t smihael/compliant-controllers:$(date +"%Y%m%d_%H%M") \
  .
docker tag smihael/compliant-controllers:$(date +"%Y%m%d_%H%M") smihael/compliant-controllers:latest 
```

Once built, I recommend using [rocker](https://github.com/osrf/rocker) for testing:
```bash
rocker --devices /dev/dri --x11 --pulse --user --network=host --home --image-name smihael/compliant-controllers:latest compliant-controllers-rocker
```

This will build a minimal overlay with necessary libraries and hacks to enable access to host window manager, mount your home directory, and forward hardware interfaces like GPU, audio, and input devices into the container for seamless desktop and device integration. You can execute further commands using `docker exec -it compliant-controllers-rocker bash`.

## gz_ros2_control adjustments 

Upstream gazebo configurations for lbr stack and UR stack exclude effort (torque) command interfaces due to limitations in `gz_ros2_control` (see [ros-controls/gz_ros2_control#182](https://github.com/ros-controls/gz_ros2_control/issues/182)) and https://github.com/ros-controls/gz_ros2_control/issues/343. Franka does this already in their upstream configuration by exposing gazebo_effort flag.

This means controllers that require **both position and effort command interfaces** cannot be used in Gazebo simulation. However, it works when just one interface per joint is used.

- LBR (IIWA/MED) Gazebo: Workaround for gz_ros2_control not exposing effort in upstream launch. Generates URDF with `mode:=mock`, swaps `mock_components/GenericSystem` for `gz_ros2_control/GazeboSimSystem`, and injects plugin config.
- UR Gazebo: Adds `<command_interface name="effort"/>` to all six joints at runtime and rewrites `ign_ros2_control/IgnitionSystem` to `gz_ros2_control/GazeboSimSystem`.

## Launch Recipes

- FR3 (real robot):

```bash
ros2 launch compliant_controllers fr3.launch.py controller_name:=cartesian_impedance_controller
```

- FR3 in Gazebo (ros_gz_sim): defaults `show_gazebo_gui:=false`, `show_rviz:=true`.

```bash
ros2 launch compliant_controllers fr3_gz.launch.py \
  controller_name:=cartesian_impedance_controller \
  show_gazebo_gui:=true \
  publish_world_to_base:=true
```

- UR (upstream ur_description, version 2.9.0, Gazebo Sim): adds `<command_interface name="effort"/>` to all six joints and rewrites `ign_ros2_control/IgnitionSystem` to `gz_ros2_control/GazeboSimSystem`. Defaults: `ur_type:=ur5e`, `controllers_file:=ur_gz_controllers.yaml`, `initial_joint_controller:=cartesian_impedance_controller`, `launch_rviz:=true`, `gazebo_gui:=true`.

```bash
ros2 launch compliant_controllers ur_gz.launch.py ur_type:=ur10e

# Headless and alternate controller examples
ros2 launch compliant_controllers ur_gz.launch.py ur_type:=ur10e launch_rviz:=false gazebo_gui:=false
ros2 launch compliant_controllers ur_gz.launch.py ur_type:=ur10e initial_joint_controller:=joint_trajectory_controller
```

- LBR (IIWA/MED) in Gazebo with effort command interfaces: generates URDF with `mode:=mock`, swaps `mock_components/GenericSystem` for `gz_ros2_control/GazeboSimSystem`, and injects plugin config. Defaults: `model:=iiwa14`, `ctrl:=iiwa14_arm_controller`, controller config `config/lbr_gz_controllers.yaml`.

```bash
ros2 launch compliant_controllers lbr_gazebo.launch.py model:=iiwa7 ctrl:=cartesian_impedance_controller
ros2 launch compliant_controllers lbr_gazebo.launch.py model:=med14 ctrl:=lbr_torque_command_controller log_level:=debug
```

### Verify Controllers and Interfaces

- UR / FR3 Gazebo: controller manager at `/controller_manager`

```bash
ros2 control list_hardware_interfaces --controller-manager /controller_manager
ros2 control list_controllers --controller-manager /controller_manager
```

- LBR Gazebo: controller manager is namespaced by `robot_name`/`model` (default `lbr`)

```bash
ros2 control list_hardware_interfaces --controller-manager /lbr/controller_manager
ros2 control list_controllers --controller-manager /lbr/controller_manager
```


### Testing Cartesian Commands

A flexible test script is provided that supports namespace and frame naming:

**Basic usage (auto-discovers frames from the controller):**
```bash
ros2 run compliant_controllers test_cartesian_command.py
```

**Explicit examples:**

- UR (Gazebo):

```bash
ros2 run compliant_controllers test_cartesian_command.py \
  --ros-args -r __ns:=/ur \
  -p controller_name:=cartesian_impedance_controller \
  -p base_frame:=base_link -p ee_frame:=tool0 \
  -p dz:=-0.02
```

- FR3 (Gazebo):

```bash
ros2 run compliant_controllers test_cartesian_command.py \
  --ros-args -r __ns:=/fr3 \
  -p controller_name:=cartesian_impedance_controller \
  -p base_frame:=panda_link0 -p ee_frame:=panda_link8 \
  -p dz:=-0.05
```

- LBR (Gazebo):

```bash
ros2 run compliant_controllers test_cartesian_command.py \
  --ros-args -r __ns:=/iiwa14 \
  -p controller_name:=cartesian_impedance_controller \
  -p base_frame:=iiwa14_link_0 -p ee_frame:=iiwa14_link_ee \
  -p dz:=-0.05
```

**Test script parameters:**
- `robot_name` (string): Robot name for default frame generation. Default: `lbr`
- `robot_ns` (string): Alternative namespace specification (prefer using `--ros-args -r __ns:=...`)
- `controller_name` (string): Controller name to query ee_frame from. Default: `cartesian_impedance_controller`
- `base_frame` (string): Base frame for TF lookup. Default: `<robot_name>_link_0`
- `ee_frame` (string): End-effector frame. Auto-discovered from controller if not set, or defaults to `<robot_name>_link_ee`
- `tf_prefix` (string): Prefix to prepend to frame names (e.g., for namespaced TFs). Default: empty
- `tf_prefix_delim` (string): Delimiter between prefix and frame name (`/` or `_`). Default: `/`
- `dx`, `dy`, `dz` (double): Cartesian position offsets [m]. Default: `0.0, 0.0, 0.05`
- `k_lin`, `k_rot` (double): Linear and rotational stiffness. Default: `300.0, 20.0`


