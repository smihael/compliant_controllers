# Generic launch include for compliant_controllers/GenericCartesianControllerWrapper.

import os
import tempfile
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


def joint_names(arm_id):
    return [f'{arm_id}_joint{i}' for i in range(1, 8)]


def explicit_joint_names(context, arm_id):
    joints = LaunchConfiguration('joints').perform(context).strip()
    if not joints:
        return joint_names(arm_id)
    return [joint.strip() for joint in joints.split(',') if joint.strip()]


def default_ee_frame(arm_id):
    return f'{arm_id}_link8'


def default_base_frame(arm_id):
    return f'{arm_id}_link0'


def as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def write_controller_overrides(context):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    arm_id = LaunchConfiguration('arm_id').perform(context)
    ee_frame = LaunchConfiguration('ee_frame').perform(context) or default_ee_frame(arm_id)
    base_frame = LaunchConfiguration('base_frame').perform(context) or default_base_frame(arm_id)
    robot_description_node = LaunchConfiguration('robot_description_node').perform(context)
    end_effector_profile_node = LaunchConfiguration('end_effector_profile_node').perform(context)

    params = {
        'arm_id': arm_id,
        'init_k_pos': float(LaunchConfiguration('init_k_pos').perform(context)),
        'init_k_ori': float(LaunchConfiguration('init_k_ori').perform(context)),
        'ee_frame': ee_frame,
        'base_frame': base_frame,
        'robot_description_node': robot_description_node,
        'robot_description_param': LaunchConfiguration('robot_description_param').perform(context),
        'end_effector_profile_node': end_effector_profile_node,
        'compensate_end_effector_load': as_bool(LaunchConfiguration('compensate_end_effector_load').perform(context)),
        'add_gravity_compensation': as_bool(LaunchConfiguration('add_gravity_compensation').perform(context)),
        'add_friction_compensation': as_bool(LaunchConfiguration('add_friction_compensation').perform(context)),
        'impl_library': LaunchConfiguration('impl_library').perform(context),
        'joints': explicit_joint_names(context, arm_id),
        'friction_compensation': {
            'model': LaunchConfiguration('friction_model').perform(context),
            'scale': float(LaunchConfiguration('friction_scale').perform(context)),
            'use_gating': as_bool(LaunchConfiguration('friction_use_gating').perform(context)),
        },
        'diagnostic_logger': {
            'log_file': LaunchConfiguration('diagnostic_log_file').perform(context),
            'duration': float(LaunchConfiguration('diagnostic_log_duration').perform(context)),
            'mode': int(LaunchConfiguration('diagnostic_mode').perform(context)),
        },
    }

    plugin_params_file = LaunchConfiguration('plugin_params_file').perform(context)
    csv_file = LaunchConfiguration('csv_file').perform(context)
    robot_profile = LaunchConfiguration('robot_profile').perform(context)
    if plugin_params_file:
        params['plugin_params_file'] = plugin_params_file
    if csv_file:
        params['csv_file'] = csv_file
    if robot_profile:
        params['robot_profile'] = robot_profile

    key = f'/{namespace}/{controller_name}' if namespace else f'/{controller_name}'
    payload = {key: {'ros__parameters': params}}
    out_path = os.path.join(tempfile.gettempdir(), f'compliant_controllers_{controller_name}_overrides.yaml')
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(payload, f, sort_keys=False)
    return out_path


def launch_runtime_actions(context, *args, **kwargs):
    override_file = write_controller_overrides(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    arm_id = LaunchConfiguration('arm_id').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context) or default_base_frame(arm_id)
    controller_manager = LaunchConfiguration('controller_manager').perform(context)
    if not controller_manager:
        controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'
    start_controller = as_bool(LaunchConfiguration('start_controller').perform(context))

    end_effector_profile_server = Node(
        package='compliant_controllers',
        executable='load_endeffector_profile.py',
        name='end_effector_profile_server',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=[LaunchConfiguration('end_effector_profile')],
        condition=IfCondition(LaunchConfiguration('load_end_effector_profile')),
    )

    spawner_arguments = [
        LaunchConfiguration('controller_name'),
        '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
        '--controller-manager', controller_manager,
        '--param-file', override_file,
    ]
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
        namespace=LaunchConfiguration('namespace'),
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
                'print(f"[generic_controller_wrapper.launch] waiting for output: {p}", flush=True)\n'
                'try:\n'
                '    while p:\n'
                '        if os.path.exists(p) and os.path.getmtime(p) >= (t0 - 0.5):\n'
                '            break\n'
                '        time.sleep(0.2)\n'
                'except KeyboardInterrupt:\n'
                '    sys.exit(0)\n'
                'print("[generic_controller_wrapper.launch] output detected", flush=True)\n'
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

    return [end_effector_profile_server, spawner, static_tf_node, completion_watcher, shutdown_on_completion]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('controller_manager', default_value=''),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        DeclareLaunchArgument('impl_library', default_value='libcartesian_impedance_impl.so'),
        DeclareLaunchArgument('init_k_pos', default_value='200.0'),
        DeclareLaunchArgument('init_k_ori', default_value='10.0'),
        DeclareLaunchArgument('joints', default_value=''),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('robot_description_node', default_value='robot_state_publisher'),
        DeclareLaunchArgument('robot_description_param', default_value='robot_description'),
        DeclareLaunchArgument('end_effector_profile_node', default_value='end_effector_profile_server'),
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
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('load_end_effector_profile', default_value='true'),
        DeclareLaunchArgument('end_effector_profile', default_value=''),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_runtime_actions)])
