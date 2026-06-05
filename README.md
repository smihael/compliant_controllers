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
- Friction estimation implementation: `FrictionEstimationImpl` (loaded via `impl_library`) performs per-joint sinusoidal excitation and logs measurement data to CSV.

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

## Friction Estimation Measurement Plugin

Use the same wrapper plugin, but set:

```yaml
impl_library: libfriction_estimation_impl.so
```

An example controller entry is included in `config/fr3_controllers.yaml` as `friction_estimation_controller`.

At runtime, the plugin now runs the complete friction pipeline internally (no external orchestration script required):

1. Move robot to `q_init`.
2. Settle at `q_init`.
3. Excite one joint while holding all others at `q_init`.
4. Return to `q_init` and settle.
5. Repeat for each selected joint.
6. Export raw samples and estimated friction parameter matrices.

The excitation follows the `friction_estimation.mdl` Motion Gen structure:

1. Four sine components with amplitudes `pi/16`, `pi/8`, `pi/4`, `pi/2` at base frequency 1 Hz (`2*pi`).
2. Each component is gated by a saturating ramp with starts `[1, 2.5, 5, 10]` and slopes `[1, 0.5, 0.25, 0.1]`.
3. Joint-specific scaling from the MATLAB function block is applied (`joint 2: x0.5`, `joint 4/6: x2/3`, others unchanged).
4. The scalar profile is composed into a 7D vector by writing only the selected joint entry.

CSV columns:

```text
t,active_joint,excitation,q,dq,tau_measured,tau_commanded,q_des,dq_des,sine_tau_ff
```

At the end of the run, the plugin also writes:

- `model1_estimated_friction.csv`: columns `joint,fv,fc,fo`
- `model2_estimated_friction.csv`: columns `joint,hysteresis,fv_p,fc_p,fv_n,fc_n`
- `friction_estimated_friction.m`: MATLAB-ready variables `q_init`, `model1_estimated_friction`, `model2_estimated_friction`

Configure via environment variables before launch:

```bash
export TEST_JOINT_INDEX=7
# Optional alias: FRICTION_ESTIMATION_TEST_JOINT_INDEX
export FRICTION_ESTIMATION_OUTPUT_DIR=/tmp
# Optional explicit file path override:
# export FRICTION_ESTIMATION_OUTPUT=/tmp/friction_results_joint7_custom.csv
export FRICTION_ESTIMATION_Q_INIT="0,-0.785398,0,-2.356194,0,1.570796,0.785398"
export FRICTION_ESTIMATION_START_JOINT=1
export FRICTION_ESTIMATION_END_JOINT=7
export FRICTION_ESTIMATION_SETTLE_SEC=1.0
export FRICTION_ESTIMATION_RETURN_SETTLE_SEC=1.0
export FRICTION_ESTIMATION_MEASURE_SEC=20.0
export FRICTION_ESTIMATION_AMPLITUDE_RAD=0.25
# Optional per-joint overrides (7 values):
# export FRICTION_ESTIMATION_AMPLITUDES_RAD="0.25,0.2,0.25,0.2,0.2,0.2,0.2"
# export FRICTION_ESTIMATION_DURATIONS_SEC="20,20,20,20,20,20,20"
export FRICTION_ESTIMATION_FREQ_HZ=0.25
export FRICTION_ESTIMATION_TAU_FF_NM=0.0
export FRICTION_ESTIMATION_HOLD_KP=50.0
export FRICTION_ESTIMATION_HOLD_KD=5.0
export FRICTION_ESTIMATION_TAU_LIMIT_NM=20.0
# Optional explicit outputs:
# export FRICTION_ESTIMATION_MODEL1_OUTPUT=/tmp/model1_estimated_friction.csv
# export FRICTION_ESTIMATION_MODEL2_OUTPUT=/tmp/model2_estimated_friction.csv
# export FRICTION_ESTIMATION_MATLAB_OUTPUT=/tmp/friction_estimated_friction.m
```

Launch with:

```bash
ros2 launch compliant_controllers fr3.launch.py controller_name:=friction_estimation_controller
```

or use the dedicated launch file with direct friction arguments:

```bash
ros2 launch compliant_controllers fr3_friction_estimation.launch.py \
  robot_ip:=192.168.1.1 \
  start_joint:=1 end_joint:=7 \
  q_init:="0,-0.785398,0,-2.356194,0,1.570796,0.785398" \
  settle_sec:=1.0 return_settle_sec:=1.0 measure_sec:=20.0 \
  move_max_vel_rad_s:=0.08 move_tau_limit_nm:=4.0 \
  move_kp_scale:=0.15 move_kd_scale:=0.30 \
  amplitude_rad:=0.25 freq_hz:=0.25 \
  output_dir:=/tmp/friction_run
```

Per-joint timing/amplitude overrides via launch:

```bash
ros2 launch compliant_controllers fr3_friction_estimation.launch.py \
  durations_sec:="20,20,20,20,20,20,20" \
  amplitudes_rad:="0.25,0.2,0.25,0.2,0.2,0.2,0.2"
```

`plot_friction_results.m` can still be used for per-joint inspection from the generated sample CSV.


