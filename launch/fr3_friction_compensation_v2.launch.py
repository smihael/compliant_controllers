# Thin alias for the shared FR3 wrapper launch in FCiJS friction-compensation mode.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('diagnostic_log_file', default_value='/tmp/fr3_friction_compensation_diagnostics.csv'),
        DeclareLaunchArgument('diagnostic_log_duration', default_value='60.0'),
    ]

    include_shared = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('compliant_controllers'),
                'launch',
                'fr3.launch.py',
            ])
        ),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'arm_id': LaunchConfiguration('arm_id'),
            'namespace': LaunchConfiguration('namespace'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
            'controller_name': 'friction_compensation_controller_v2',
            'impl_library': 'libcartesian_impedance_impl.so',
            'init_k_pos': '0.0',
            'init_k_ori': '0.0',
            'add_gravity_compensation': 'false',
            'compensate_end_effector_load': 'false',
            'add_friction_compensation': 'true',
            'friction_model': 'fcijs',
            'friction_scale': '1.0',
            'friction_use_gating': 'true',
            'diagnostic_log_file': LaunchConfiguration('diagnostic_log_file'),
            'diagnostic_log_duration': LaunchConfiguration('diagnostic_log_duration'),
            'diagnostic_mode': '2',
        }.items(),
    )

    return LaunchDescription(declared_args + [include_shared])
