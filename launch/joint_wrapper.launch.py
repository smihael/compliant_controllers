# Launch include for compliant_controllers/GenericJointControllerWrapper.

import json

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    controller_manager = LaunchConfiguration('controller_manager').perform(context)
    if not controller_manager:
        controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'

    controller_name = LaunchConfiguration('controller_name').perform(context)
    base_spawner_arguments = [
        LaunchConfiguration('controller_name'),
        '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
        '--controller-manager', controller_manager,
    ]
    parameters = selected_profile_parameters(
        LaunchConfiguration('robot_profile_file').perform(context),
        LaunchConfiguration('friction_compensation_profile').perform(context))
    parameters.pop('dithering_enabled', None)
    friction_enabled = LaunchConfiguration('friction_compensation_enabled').perform(context)
    if friction_enabled != '':
        parameters['friction_compensation_enabled'] = as_bool(friction_enabled)

    start_controller = as_bool(LaunchConfiguration('start_controller').perform(context))
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
    handler = RegisterEventHandler(
        OnProcessExit(target_action=load_spawner, on_exit=[configure_process]))
    return [load_spawner, handler]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('robot_profile_file', default_value=''),
        DeclareLaunchArgument(
            'friction_compensation_profile',
            default_value='friction_compensation_sigmoid'),
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
        DeclareLaunchArgument('friction_compensation_enabled', default_value=''),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_runtime_actions)])
