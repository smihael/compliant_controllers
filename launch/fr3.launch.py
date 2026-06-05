# Real FR3/Panda robot launch with GenericCartesianControllerWrapper.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_robot_profile(defaults, profile_name):
    profile = defaults.copy()
    config_path = os.path.join(
        get_package_share_directory('compliant_controllers'),
        'config',
        'robot_config.yaml',
    )
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f) or {}
        if isinstance(loaded, dict):
            selected = loaded.get(profile_name)
            if isinstance(selected, dict):
                profile.update({k: v for k, v in selected.items() if v is not None})
            else:
                profile.update({k: v for k, v in loaded.items() if v is not None})
    except Exception:
        pass
    return profile


def generate_launch_description():
    robot_profile = os.environ.get('COMPLIANT_ROBOT_PROFILE', 'ROBOT_1')
    robot_cfg = load_robot_profile({
        'robot_name': 'fr3',
        'robot_ip': '192.168.1.1',
    }, robot_profile)

    default_end_effector_profile = os.path.join(
        get_package_share_directory('compliant_controllers'),
        'config',
        'franka_hand_default.endeffector-profile.json',
    )

    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=robot_profile),
        DeclareLaunchArgument('robot_type', default_value=str(robot_cfg.get('robot_type', robot_cfg.get('robot_name', 'fr3')))),
        DeclareLaunchArgument('arm_id', default_value=str(robot_cfg.get('robot_name', 'fr3'))),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value=str(robot_cfg.get('robot_ip', '192.168.1.1'))),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('impl_library', default_value='libcartesian_impedance_impl.so'),
        DeclareLaunchArgument('init_k_pos', default_value='200.0'),
        DeclareLaunchArgument('init_k_ori', default_value='10.0'),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('add_gravity_compensation', default_value='false'),
        DeclareLaunchArgument('compensate_end_effector_load', default_value='false'),
        DeclareLaunchArgument('add_friction_compensation', default_value='false'),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
        DeclareLaunchArgument('plugin_params_file', default_value=''),
        DeclareLaunchArgument('csv_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_duration', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_mode', default_value='0'),
        DeclareLaunchArgument('shutdown_on_done', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('load_end_effector_profile', default_value='true'),
        DeclareLaunchArgument('end_effector_profile', default_value=default_end_effector_profile),
    ]

    include_franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])
        ),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'arm_id': LaunchConfiguration('arm_id'),
            'arm_prefix': LaunchConfiguration('arm_prefix'),
            'namespace': LaunchConfiguration('namespace'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'controllers_yaml': PathJoinSubstitution([
                FindPackageShare('compliant_controllers'), 'config', 'fr3_controllers.yaml'
            ]),
        }.items(),
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('compliant_controllers'),
                'launch',
                'generic_controller_wrapper.launch.py',
            ])
        ),
        launch_arguments={
            'robot_profile': LaunchConfiguration('robot_profile'),
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'impl_library': LaunchConfiguration('impl_library'),
            'init_k_pos': LaunchConfiguration('init_k_pos'),
            'init_k_ori': LaunchConfiguration('init_k_ori'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'robot_description_node': 'robot_state_publisher',
            'robot_description_param': 'robot_description',
            'end_effector_profile_node': 'end_effector_profile_server',
            'add_gravity_compensation': LaunchConfiguration('add_gravity_compensation'),
            'compensate_end_effector_load': LaunchConfiguration('compensate_end_effector_load'),
            'add_friction_compensation': LaunchConfiguration('add_friction_compensation'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'plugin_params_file': LaunchConfiguration('plugin_params_file'),
            'csv_file': LaunchConfiguration('csv_file'),
            'diagnostic_log_file': LaunchConfiguration('diagnostic_log_file'),
            'diagnostic_log_duration': LaunchConfiguration('diagnostic_log_duration'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'shutdown_on_done': LaunchConfiguration('shutdown_on_done'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
            'load_end_effector_profile': LaunchConfiguration('load_end_effector_profile'),
            'end_effector_profile': LaunchConfiguration('end_effector_profile'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('namespace'),
        arguments=['--display-config', PathJoinSubstitution([
            FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(declared_args + [include_franka, include_controller, rviz_node])
