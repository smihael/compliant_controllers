# Launch include for compliant_controllers/GenericJointControllerWrapper.

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
    declared_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('controller_name', default_value='joint_impedance_controller'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('controller_manager', default_value=''),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        DeclareLaunchArgument('impl_library', default_value='libjoint_impedance_impl.so'),
        DeclareLaunchArgument('joints', default_value=''),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('robot_description_node', default_value='robot_state_publisher'),
        DeclareLaunchArgument('robot_description_param', default_value='robot_description'),
        DeclareLaunchArgument('initial_stiffness', default_value='600,600,600,600,250,150,50'),
        DeclareLaunchArgument('initial_damping', default_value='30,30,30,30,10,10,5'),
        DeclareLaunchArgument('filter_alpha', default_value='0.99'),
        DeclareLaunchArgument('max_tau_delta', default_value='1.0'),
        DeclareLaunchArgument('power_enable_tau_norm_threshold', default_value='1.1'),
        DeclareLaunchArgument('max_power_enable_count', default_value='100'),
        DeclareLaunchArgument('friction_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_runtime_actions)])
