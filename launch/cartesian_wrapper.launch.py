# Launch include for compliant_controllers/GenericCartesianControllerWrapper.

import json

import yaml

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

def flatten_parameters(prefix, value, output):
    if isinstance(value, dict):
        for key, child in value.items():
            flatten_parameters(f'{prefix}.{key}' if prefix else key, child, output)
    else:
        output[prefix] = value


def selected_profile_parameters(path, profile_name):
    if not path or not profile_name:
        return {}
    with open(path, 'r', encoding='utf-8') as source:
        document = yaml.safe_load(source) or {}
    selected = document.get(profile_name)
    if not isinstance(selected, dict) or 'ros__parameters' not in selected:
        raise RuntimeError(
            f"Friction compensation profile '{profile_name}' is missing from {path}")
    parameters = {}
    flatten_parameters('', selected['ros__parameters'], parameters)
    return parameters


def launch_runtime_actions(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    arm_id = LaunchConfiguration('arm_id').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context) or default_base_frame(arm_id)
    controller_manager = LaunchConfiguration('controller_manager').perform(context)
    if not controller_manager:
        controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'
    start_controller = as_bool(LaunchConfiguration('start_controller').perform(context))

    base_spawner_arguments = [
        LaunchConfiguration('controller_name'),
        '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
        '--controller-manager', controller_manager,
    ]
    parameters = selected_profile_parameters(
        LaunchConfiguration('robot_profile_file').perform(context),
        LaunchConfiguration('friction_compensation_profile').perform(context))
    friction_enabled = LaunchConfiguration('friction_compensation_enabled').perform(context)
    dithering_enabled = LaunchConfiguration('dithering_enabled').perform(context)
    gravity_enabled = LaunchConfiguration('gravity_compensation_enabled').perform(context)
    guard_enabled = LaunchConfiguration('max_step_guard_enabled').perform(context)
    log_file = LaunchConfiguration('log_file').perform(context)
    plugin_params_file = LaunchConfiguration('plugin_params_file').perform(context)
    if friction_enabled != '':
        parameters['friction_compensation_enabled'] = as_bool(friction_enabled)
    if dithering_enabled != '':
        parameters['dithering_enabled'] = as_bool(dithering_enabled)
    if gravity_enabled != '':
        parameters['gravity_compensation_enabled'] = as_bool(gravity_enabled)
    if guard_enabled != '':
        parameters['max_step_guard_enabled'] = as_bool(guard_enabled)
    if log_file:
        parameters['log_file'] = log_file
    if plugin_params_file:
        parameters['plugin_params_file'] = plugin_params_file

    load_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=base_spawner_arguments + ['--load-only'],
        output='screen',
    )
    target = f'/{namespace}/{controller_name}' if namespace else f'/{controller_name}'
    configure_process = ExecuteProcess(cmd=[
        'python3', '-c',
        (
            'import json, sys, rclpy\n'
            'from rclpy.node import Node\n'
            'from rclpy.parameter import Parameter\n'
            'from rcl_interfaces.srv import SetParameters\n'
            'from controller_manager import configure_controller, switch_controllers\n'
            'rclpy.init()\n'
            'node = Node("controller_parameter_configurator")\n'
            'values = json.loads(sys.argv[1])\n'
            'client = node.create_client(SetParameters, sys.argv[2] + "/set_parameters")\n'
            'assert client.wait_for_service(timeout_sec=10.0), "parameter service unavailable"\n'
            'request = SetParameters.Request()\n'
            'request.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]\n'
            'future = client.call_async(request)\n'
            'rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)\n'
            'result = future.result()\n'
            'assert result is not None and all(r.successful for r in result.results), result\n'
            'assert configure_controller(node, sys.argv[3], sys.argv[4], 10.0, 10.0).ok\n'
            'if sys.argv[5] == "true":\n'
            '  assert switch_controllers(node, sys.argv[3], [], [sys.argv[4]], True, True, 5.0, 10.0).ok\n'
            'rclpy.shutdown()\n'
        ), json.dumps(parameters), target, controller_manager, controller_name,
        'true' if start_controller else 'false'], output='screen')
    configure_handler = RegisterEventHandler(
        OnProcessExit(target_action=load_spawner, on_exit=[configure_process]))

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
            LaunchConfiguration('log_file'),
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

    return [load_spawner, configure_handler, static_tf_node]
     #,           completion_watcher, shutdown_on_completion]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=''),
        DeclareLaunchArgument('robot_profile_file', default_value=''),
        DeclareLaunchArgument(
            'friction_compensation_profile',
            default_value='friction_compensation_sigmoid'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('arm_id', default_value=''),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('controller_manager', default_value=''),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        DeclareLaunchArgument('impl_library', default_value=''),
        DeclareLaunchArgument('joints', default_value=''),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('robot_description_node', default_value=''),
        DeclareLaunchArgument('robot_description_param', default_value=''),
        DeclareLaunchArgument('gravity_compensation_enabled', default_value=''),
        DeclareLaunchArgument('dithering_enabled', default_value=''),
        DeclareLaunchArgument('friction_compensation_enabled', default_value=''),
        DeclareLaunchArgument('max_step_guard_enabled', default_value=''),
        DeclareLaunchArgument('plugin_params_file', default_value=''),
        DeclareLaunchArgument('log_file', default_value=''),
        DeclareLaunchArgument('shutdown_on_done', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_runtime_actions)])
