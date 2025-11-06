# compliant_controllers

Robot-agnostic compliant controllers for ROS 2 using ros2_control. 

## Architecture & features
- ROS-free core abstract classes:
	- `RobotModel`: defines interface for computing J(q), C(q, dq), and M(q)
	- `ControlCommand`, `ControlStates`: lightweight data containers for commands and feedback.
	- `AbstractController`: defines interface for computing joint torques based on the above data containers.
- Sample robot model implementation based on the Pinocchio library that reads the URDF from ROS parameters
- Sample controller implementation: `CartesianImpedanceController : AbstractController` builds as a plain shared library (.so) without ROS deps
- ROS 2 wrapper: `GenericCartesianControllerWrapper` is a ros2_control plugin that instantiates the chosen `AbstractController`, streams states in real time, calls a `RobotModel` implementation and the controller’s `step(...)`, and writes torques.

Thus, the controller logic is fully separated from ROS and the robot. This keeps controller code simple and readable (no ROS 2 boilerplate). Controller implementations can be written in C++ or e.g., Simulink (WIP).

## Build
```
colcon build --packages-select compliant_controllers
source install/setup.bash
```

## Usage

Example for FR3:

```bash
ros2 launch franka_bringup example.launch.py controller_name:=cartesian_impedance_controller
```

Example for FR3 in Gazebo:

```bash
ros2 launch compliant_controllers gazebo.launch.py show_gazebo_gui:=true
```

A simple test script is provided:

```bash
ros2 run compliant_controllers test_cartesian_command.py --ros-args -p dz:=-0.05
```

## ROS parameters
The controller accepts the following parameters (namespaced under your controller instance):

- arm_id (string): Joint name prefix (e.g., fr3, panda). Default: fr3.
- init_k_pos (double): Initial Cartesian position stiffness [N/m]. Default: 200.0.
- init_k_ori (double): Initial Cartesian orientation stiffness [Nm/rad]. Default: 10.0.
- ee_frame (string): End-effector frame used by the controller. Default: fr3_link8.
- robot_description_node (string): Node that provides the robot_description parameter. Default: robot_state_publisher.
- robot_description_param (string): Parameter name containing the URDF. Default: robot_description.
- impl_library (string): Path to the shared library (.so) providing the AbstractController implementation to load (e.g., libcartesian_impedance_impl.so).
