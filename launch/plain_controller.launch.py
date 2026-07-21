# Minimal launch include for spawning an already-configured ros2_control controller.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_runtime_actions(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    controller_manager = LaunchConfiguration('controller_manager').perform(context)
    if not controller_manager:
        controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'

    spawner_arguments = [
        LaunchConfiguration('controller_name'),
        '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
        '--controller-manager', controller_manager,
    ]
    if not as_bool(LaunchConfiguration('start_controller').perform(context)):
        spawner_arguments.append('--stopped')

    return [Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=spawner_arguments,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('controller_manager', default_value=''),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        OpaqueFunction(function=launch_runtime_actions),
    ])
