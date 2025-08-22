#!/usr/bin/env python3
"""
One-shot launch: fake Franka + CartesianImpedanceController.

Usage:
  ros2 launch compliant_controllers cartesian_impedance_fake.launch.py
Optional overrides:
  ros2 launch compliant_controllers cartesian_impedance_fake.launch.py arm_id:=fr3 k_p:=300.0 k_d:=15.0 namespace:=demo

Hard-coded:
  robot_ip:=dont-care
  use_fake_hardware:=true

Requires: franka_bringup, compliant_controllers built & sourced.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    arm_id = LaunchConfiguration('arm_id').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    k_p = float(LaunchConfiguration('k_p').perform(context))
    k_d = float(LaunchConfiguration('k_d').perform(context))

    franka_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
            ])
        ),
        launch_arguments={
            'arm_id': arm_id,
            'arm_prefix': '',
            'namespace': namespace,
            'urdf_file': 'fr3/fr3.urdf.xacro',
            'robot_ip': 'dont-care',
            'load_gripper': 'false',
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'false',
            'joint_state_rate': '30',
        }.items()
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'cartesian_impedance_controller',
            '--controller-manager-timeout', '60',
            '--controller-type', 'compliant_controllers/CartesianImpedanceController'
        ],
        parameters=[{'arm_id': arm_id, 'k_p': k_p, 'k_d': k_d}],
        output='screen'
    )

    return [franka_core, controller_spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_id', default_value='fr3', description='Arm model ID.'),
        DeclareLaunchArgument('namespace', default_value='', description='Optional namespace.'),
        DeclareLaunchArgument('k_p', default_value='200.0', description='Uniform proportional gain.'),
        DeclareLaunchArgument('k_d', default_value='10.0', description='Uniform derivative gain.'),
        OpaqueFunction(function=launch_setup)
    ])
