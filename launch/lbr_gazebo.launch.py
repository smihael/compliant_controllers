"""Launch file for LBR robot in Gazebo with effort command interfaces enabled.

This launch file enables effort command interfaces in Gazebo mode by post-processing
the URDF to include effort command interfaces alongside position interfaces.

The LBR stack's Gazebo configuration (humble branch November 2025, https://github.com/lbr-stack/lbr_fri_ros2_stack/commit/2e932d4e062966afe216710a7cc2fe50a8099365)
excludes effort interfaces due to gz_ros2_control limitations, see:
https://github.com/ros-controls/gz_ros2_control/issues/182
https://github.com/ros-controls/gz_ros2_control/issues/343

This modified version:
1. Generates URDF with mode:=mock (includes effort interfaces)  
2. Replaces mock_components plugin with gz_ros2_control plugin via XML processing

Usage:
    ros2 launch compliant_controllers lbr_gazebo.launch.py \
        model:=iiwa14 \
        ctrl:=cartesian_impedance_controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin  
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
import subprocess
import xml.etree.ElementTree as ET


def generate_robot_description_with_effort(context, *args, **kwargs):
    """Generate robot description with effort command interfaces for Gazebo.
    
    This processes the URDF by:
    1. Generating with mode:=mock to include effort command interfaces
    2. Replacing mock_components/GenericSystem with gz_ros2_control/GazeboSimSystem
    3. Ensuring proper Gazebo plugin configuration exists
    
    Returns a list containing the robot_state_publisher node with modified URDF.
    """
    
    model = LaunchConfiguration("model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    sys_cfg_pkg = LaunchConfiguration("sys_cfg_pkg").perform(context)
    init_jnt_pos = LaunchConfiguration("init_jnt_pos").perform(context)
    
    # Resolve paths
    xacro_file = PathJoinSubstitution([
        FindPackageShare("lbr_description"),
        "urdf", model, f"{model}.xacro"
    ]).perform(context)
    
    system_config_path = PathJoinSubstitution([
        FindPackageShare("lbr_description"),
        "ros2_control/lbr_system_config.yaml"
    ]).perform(context)
    
    initial_positions_path = PathJoinSubstitution([
        FindPackageShare(sys_cfg_pkg),
        init_jnt_pos
    ]).perform(context)
    
    # Generate URDF with mode:=mock to include effort command interfaces
    xacro_cmd = [
        "xacro", xacro_file,
        f"robot_name:={robot_name}",
        "mode:=mock",  # Critical: use mock mode to get effort interfaces
        f"system_config_path:={system_config_path}",
        f"initial_joint_positions_path:={initial_positions_path}",
    ]
    
    try:
        urdf_str = subprocess.check_output(xacro_cmd, text=True, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to generate URDF: {e.stderr}")
        raise
    
    # Parse XML and modify for Gazebo
    try:
        root = ET.fromstring(urdf_str)
    except ET.ParseError as e:
        print(f"[ERROR] Failed to parse URDF XML: {e}")
        raise
    
    # Replace mock_components plugin with gz_ros2_control plugin
    modified = False
    for ros2_control in root.findall('.//ros2_control'):
        hardware = ros2_control.find('hardware')
        if hardware is not None:
            plugin = hardware.find('plugin')
            if plugin is not None and 'mock_components' in plugin.text:
                plugin.text = 'gz_ros2_control/GazeboSimSystem'
                modified = True
                print(f"[INFO] Replaced mock_components with gz_ros2_control/GazeboSimSystem")
    
    if not modified:
        print("[WARNING] No mock_components plugin found to replace")
    
    # Ensure Gazebo ros2_control plugin configuration exists and uses correct controller config
    gazebo_plugin_found = False
    ctrl_cfg_pkg = LaunchConfiguration("ctrl_cfg_pkg").perform(context)
    ctrl_cfg = LaunchConfiguration("ctrl_cfg").perform(context)
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(ctrl_cfg_pkg),
        ctrl_cfg
    ]).perform(context)
    
    for gazebo in root.findall('.//gazebo'):
        for plugin in gazebo.findall('plugin'):
            if 'gz_ros2_control' in plugin.get('filename', ''):
                gazebo_plugin_found = True
                # Update the parameters path to use compliant_controllers config
                params_elem = plugin.find('parameters')
                if params_elem is not None:
                    params_elem.text = controllers_yaml
                    print(f"[INFO] Updated gz_ros2_control plugin to use: {controllers_yaml}")
                else:
                    print("[WARNING] gz_ros2_control plugin found but no parameters element")
                break
    
    if not gazebo_plugin_found:
        print("[INFO] Adding gz_ros2_control Gazebo plugin configuration")
        gazebo_elem = root.find('.//gazebo')
        if gazebo_elem is None:
            gazebo_elem = ET.SubElement(root, 'gazebo')
        
        plugin = ET.SubElement(gazebo_elem, 'plugin')
        plugin.set('name', 'gz_ros2_control::GazeboSimROS2ControlPlugin')
        plugin.set('filename', 'gz_ros2_control-system')
        
        params = ET.SubElement(plugin, 'parameters')
        # Use controller config from compliant_controllers package
        ctrl_cfg_pkg = LaunchConfiguration("ctrl_cfg_pkg").perform(context)
        ctrl_cfg = LaunchConfiguration("ctrl_cfg").perform(context)
        controllers_yaml = PathJoinSubstitution([
            FindPackageShare(ctrl_cfg_pkg),
            ctrl_cfg
        ]).perform(context)
        params.text = controllers_yaml
        print(f"[INFO] Using controller config: {controllers_yaml}")
        
        ros_ns = ET.SubElement(plugin, 'ros')
        namespace = ET.SubElement(ros_ns, 'namespace')
        namespace.text = f"/{robot_name}"
    
    # Convert back to string
    urdf_modified = ET.tostring(root, encoding='unicode')
    
    print(f"[INFO] Generated robot description with effort interfaces enabled for {model}")
    
    # Create robot_state_publisher node with modified URDF
    from launch_ros.actions import Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='both',
        parameters=[{
            'robot_description': urdf_modified,
            'use_sim_time': True,
        }]
    )
    
    return [robot_state_publisher]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    # Override default controller config to use compliant_controllers package
    ld.add_action(
        DeclareLaunchArgument(
            "ctrl_cfg_pkg",
            default_value="compliant_controllers",
            description="Package containing controller configuration"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "ctrl_cfg",
            default_value="config/lbr_gz_controllers.yaml",
            description="Controller configuration file"
        )
    )
    ld.add_action(LBRROS2ControlMixin.arg_init_jnt_pos())
    # Custom controller argument without validation - allow any controller from our config
    ld.add_action(
        DeclareLaunchArgument(
            "ctrl",
            default_value="cartesian_impedance_controller",
            description="Generic Cartesian controller wrapper to spawn"
        )
    )
    ld.add_action(DeclareLaunchArgument("impl_library", default_value="libcartesian_impedance_impl.so"))
    ld.add_action(DeclareLaunchArgument("init_k_pos", default_value="200.0"))
    ld.add_action(DeclareLaunchArgument("init_k_ori", default_value="10.0"))
    ld.add_action(DeclareLaunchArgument("add_gravity_compensation", default_value="true"))
    ld.add_action(DeclareLaunchArgument("compensate_end_effector_load", default_value="false"))
    ld.add_action(DeclareLaunchArgument("add_friction_compensation", default_value="false"))
    ld.add_action(DeclareLaunchArgument("friction_model", default_value="auto"))
    ld.add_action(DeclareLaunchArgument("friction_scale", default_value="1.0"))
    ld.add_action(DeclareLaunchArgument("friction_use_gating", default_value="true"))
    ld.add_action(DeclareLaunchArgument("diagnostic_log_file", default_value=""))
    ld.add_action(DeclareLaunchArgument("diagnostic_log_duration", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("diagnostic_mode", default_value="0"))
    ld.add_action(DeclareLaunchArgument("publish_world_to_base", default_value="true"))
    # Optional: Launch log level control
    ld.add_action(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal)",
            choices=["debug", "info", "warn", "error", "fatal"]
        )
    )
    
    # Additional arguments for system config
    ld.add_action(
        DeclareLaunchArgument(
            "sys_cfg_pkg",
            default_value="lbr_description",
            description="Package containing system configuration files"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "sys_cfg",
            default_value="ros2_control/lbr_system_config.yaml",
            description="System configuration file path"
        )
    )

    # Generate robot description with effort interfaces using OpaqueFunction
    # This processes the URDF at launch time to enable effort command interfaces
    robot_description_with_effort = OpaqueFunction(
        function=generate_robot_description_with_effort
    )
    ld.add_action(robot_description_with_effort)

    # Gazebo
    ld.add_action(GazeboMixin.include_gazebo())
    ld.add_action(GazeboMixin.node_clock_bridge())
    ld.add_action(GazeboMixin.node_create())

    # Controllers - spawn after Gazebo and ros2_control are ready
    # Create custom spawner nodes with log level support
    from launch_ros.actions import Node
    
    robot_name = LaunchConfiguration("robot_name")
    log_level = LaunchConfiguration("log_level")
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "controller_manager",
            "--ros-args", "--log-level", log_level
        ],
        namespace=robot_name,
    )
    ld.add_action(joint_state_broadcaster)
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("compliant_controllers"),
                "launch",
                "generic_controller_wrapper.launch.py",
            ])
        ),
        launch_arguments={
            "namespace": robot_name,
            "arm_id": "lbr",
            "controller_name": LaunchConfiguration("ctrl"),
            "controller_manager": ["/", robot_name, "/controller_manager"],
            "impl_library": LaunchConfiguration("impl_library"),
            "init_k_pos": LaunchConfiguration("init_k_pos"),
            "init_k_ori": LaunchConfiguration("init_k_ori"),
            "joints": "lbr_A1,lbr_A2,lbr_A3,lbr_A4,lbr_A5,lbr_A6,lbr_A7",
            "ee_frame": "lbr_link_ee",
            "base_frame": "lbr_link_0",
            "robot_description_node": ["/", robot_name, "/robot_state_publisher"],
            "robot_description_param": "robot_description",
            "load_end_effector_profile": "false",
            "add_gravity_compensation": LaunchConfiguration("add_gravity_compensation"),
            "compensate_end_effector_load": LaunchConfiguration("compensate_end_effector_load"),
            "add_friction_compensation": LaunchConfiguration("add_friction_compensation"),
            "friction_model": LaunchConfiguration("friction_model"),
            "friction_scale": LaunchConfiguration("friction_scale"),
            "friction_use_gating": LaunchConfiguration("friction_use_gating"),
            "diagnostic_log_file": LaunchConfiguration("diagnostic_log_file"),
            "diagnostic_log_duration": LaunchConfiguration("diagnostic_log_duration"),
            "diagnostic_mode": LaunchConfiguration("diagnostic_mode"),
            "publish_world_to_base": LaunchConfiguration("publish_world_to_base"),
        }.items(),
    )
    ld.add_action(controller)
    
    return ld
