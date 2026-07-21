# Launch include for compliant_controllers/GenericCartesianControllerWrapper.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_base_frame(arm_id):
    return f'{arm_id}_link0'


def as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_runtime_actions(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    arm_id = LaunchConfiguration('arm_id').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context) or default_base_frame(arm_id)
    controller_manager = LaunchConfiguration('controller_manager').perform(context)
    if not controller_manager:
        controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'
    start_controller = as_bool(LaunchConfiguration('start_controller').perform(context))

    spawner_arguments = [
        LaunchConfiguration('controller_name'),
        '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
        '--controller-manager', controller_manager,
    ]
    for params_file in (
            LaunchConfiguration('robot_profile_file').perform(context),
            LaunchConfiguration('plugin_params_file').perform(context)):
        if params_file:
            spawner_arguments.extend(['--param-file', params_file])
    if not start_controller:
        spawner_arguments.append('--stopped')

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=spawner_arguments,
        output='screen',
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', base_frame],
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_world_to_base')),
    )

    completion_watcher = ExecuteProcess(
        cmd=[
            'python3',
            '-c',
            (
                'import os, sys, time\n'
                'p = sys.argv[1]\n'
                't0 = time.time()\n'
                'print(f"[cartesian_wrapper.launch] waiting for output: {p}", flush=True)\n'
                'try:\n'
                '    while p:\n'
                '        if os.path.exists(p) and os.path.getmtime(p) >= (t0 - 0.5):\n'
                '            break\n'
                '        time.sleep(0.2)\n'
                'except KeyboardInterrupt:\n'
                '    sys.exit(0)\n'
                'print("[cartesian_wrapper.launch] output detected", flush=True)\n'
            ),
            LaunchConfiguration('csv_file'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('shutdown_on_done')),
    )

    shutdown_on_completion = RegisterEventHandler(
        OnProcessExit(
            target_action=completion_watcher,
            on_exit=[EmitEvent(event=Shutdown(reason='Controller workflow finished'))],
        )
    )

    return [spawner, static_tf_node, completion_watcher, shutdown_on_completion]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=''),
        DeclareLaunchArgument('robot_profile_file', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('arm_id', default_value=''),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('controller_manager', default_value=''),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        DeclareLaunchArgument('impl_library', default_value=''),
        DeclareLaunchArgument('init_k_pos', default_value=''),
        DeclareLaunchArgument('init_k_ori', default_value=''),
        DeclareLaunchArgument('joints', default_value=''),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('robot_description_node', default_value=''),
        DeclareLaunchArgument('robot_description_param', default_value=''),
        DeclareLaunchArgument('gravity_compensation_enabled', default_value=''),
        DeclareLaunchArgument('ee_load_compensation_enabled', default_value=''),
        DeclareLaunchArgument('dithering_enabled', default_value=''),
        DeclareLaunchArgument('friction_compensation_enabled', default_value=''),
        DeclareLaunchArgument('friction_model', default_value=''),
        DeclareLaunchArgument('friction_scale', default_value=''),
        DeclareLaunchArgument('friction_use_gating', default_value=''),
        DeclareLaunchArgument('plugin_params_file', default_value=''),
        DeclareLaunchArgument('csv_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_duration', default_value=''),
        DeclareLaunchArgument('diagnostic_log_filter_tag', default_value=''),
        DeclareLaunchArgument('shutdown_on_done', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_runtime_actions)])
