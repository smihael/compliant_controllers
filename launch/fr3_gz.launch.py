"""Gazebo simulation launch for FR3 robot using compliant controllers.

Mirrors real robot launch style while adding a debug flag and optional world->base TF.
"""

import os
import json
import tempfile
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _profile_to_controller_params(profile_path):
    with open(profile_path, 'r', encoding='utf-8') as handle:
        profile = json.load(handle)

    inertial = profile.get('inertial', {})
    center_of_mass = inertial.get('centerOfMass', {})
    inertia = inertial.get('inertia', {})

    def number(data, key, default=0.0):
        return float(data.get(key, default))

    return {
        'end_effector_profile.id': str(profile.get('id', '')),
        'end_effector_profile.name': str(profile.get('name', '')),
        'end_effector_profile.device_id': str(profile.get('deviceId', '')),
        'end_effector_profile.load.mass': number(inertial, 'mass'),
        'end_effector_profile.load.center_of_mass': [
            number(center_of_mass, 'x'),
            number(center_of_mass, 'y'),
            number(center_of_mass, 'z'),
        ],
        'end_effector_profile.load.inertia': [
            number(inertia, 'x11'),
            number(inertia, 'x12'),
            number(inertia, 'x13'),
            number(inertia, 'x12'),
            number(inertia, 'x22'),
            number(inertia, 'x23'),
            number(inertia, 'x13'),
            number(inertia, 'x23'),
            number(inertia, 'x33'),
        ],
        'end_effector_profile.raw_json': json.dumps(profile, separators=(',', ':')),
    }


def _controller_yaml_with_profile(base_yaml, profile_path, enabled):
    if not enabled:
        return base_yaml

    with open(base_yaml, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    controller_params = config.setdefault('/cartesian_impedance_controller', {}).setdefault('ros__parameters', {})
    controller_params.update(_profile_to_controller_params(profile_path))

    with tempfile.NamedTemporaryFile('w', encoding='utf-8', suffix='.yaml', delete=False) as handle:
        yaml.safe_dump(config, handle, sort_keys=False)
        return handle.name


def _build_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand, load_end_effector_profile, end_effector_profile):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    load_profile = context.perform_substitution(load_end_effector_profile).lower() in ('true', '1', 'yes')
    profile_path = context.perform_substitution(end_effector_profile)

    franka_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, f'{arm_id_str}.urdf.xacro'
    )
    xacro_doc = xacro.process_file(
        franka_xacro,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true',
        }
    )
    default_yaml = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'config', 'franka_gazebo_controllers.yaml'
    )
    custom_yaml = os.path.join(
        get_package_share_directory('compliant_controllers'),
        'config', 'fr3_gz_controllers.yaml'
    )
    custom_yaml = _controller_yaml_with_profile(custom_yaml, profile_path, load_profile)
    urdf_xml = xacro_doc.toxml().replace(default_yaml, custom_yaml)
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='both', parameters=[{'robot_description': urdf_xml}]
    )]


def _gazebo_include(context: LaunchContext, world, show_gazebo_gui, controller_debug):
    world_file = context.perform_substitution(world)
    gui_flag = context.perform_substitution(show_gazebo_gui).lower() in ('true', '1', 'yes')
    debug_flag = context.perform_substitution(controller_debug).lower() in ('true', '1', 'yes')

    if gui_flag:
        gz_args = f"{world_file} -r"
    else:
        verbosity = '4' if debug_flag else '2'
        gz_args = f"{world_file} -v {verbosity} -s -r --headless-rendering"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return [gazebo_launch]


def _build_runtime_nodes(context: LaunchContext, controller_name, publish_world_to_base, show_rviz, controller_debug):
    controller = context.perform_substitution(controller_name)
    publish_tf = context.perform_substitution(publish_world_to_base).lower() in ('true', '1', 'yes')
    rviz_flag = context.perform_substitution(show_rviz).lower() in ('true', '1', 'yes')
    debug_flag = context.perform_substitution(controller_debug).lower() in ('true', '1', 'yes')

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_fr3',
        arguments=['-topic', '/robot_description'], output='screen'
    )
    jsb_args = ['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    main_args = [controller, '--controller-manager', '/controller_manager']
    if debug_flag:
        main_args.extend(['--ros-args', '--log-level', 'DEBUG'])

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_jsb', output='screen',
        arguments=jsb_args
    )
    main_controller_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_primary_controller', output='screen',
        arguments=main_args
    )

    # Start state broadcaster and primary controller in parallel right after spawn
    # to minimize the no-controller startup window (reduces initial sag).
    spawn_to_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner, main_controller_spawner])
    )

    nodes = [spawn, spawn_to_controllers]

    # Static TF world -> panda_link0 (base frame)
    if publish_tf:
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_tf_world_to_base',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'panda_link0',
            ],
            output='screen'
        ))

    # RViz
    if rviz_flag:
        rviz_config = os.path.join(get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['--display-config', rviz_config, '-f', 'world']
        ))

    # Joint state publisher
    #nodes.append(Node(
    #    package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher',
    #    parameters=[{'source_list': ['joint_states'], 'rate': 30}], output='screen'
    #))

    return nodes


def generate_launch_description():
    default_end_effector_profile = os.path.join(
        get_package_share_directory('compliant_controllers'),
        'config',
        'franka_hand_default.endeffector-profile.json'
    )

    declared_args = [
        DeclareLaunchArgument('arm_id', default_value='fr3', description='Arm identifier'),
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace (unused in sim)'),
        DeclareLaunchArgument('load_gripper', default_value='false', description='Load gripper in URDF'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand', description='Gripper variant'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller', description='Primary controller to spawn'),
        DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world file'),
        DeclareLaunchArgument('show_gazebo_gui', default_value='false', description='Show Gazebo GUI'),
        DeclareLaunchArgument('show_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true', description='Publish static world->base TF'),
        DeclareLaunchArgument('controller_debug', default_value='false', description='Enable debug logging for controller_manager (Gazebo process)'),
        DeclareLaunchArgument('load_end_effector_profile', default_value='true', description='Start end-effector profile parameter server'),
        DeclareLaunchArgument('end_effector_profile', default_value=default_end_effector_profile, description='Path to end-effector profile JSON'),
    ]

    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')
    world = LaunchConfiguration('world')
    show_gazebo_gui = LaunchConfiguration('show_gazebo_gui')
    controller_debug = LaunchConfiguration('controller_debug')
    controller_name = LaunchConfiguration('controller_name')
    publish_world_to_base = LaunchConfiguration('publish_world_to_base')
    show_rviz = LaunchConfiguration('show_rviz')
    load_end_effector_profile = LaunchConfiguration('load_end_effector_profile')
    end_effector_profile = LaunchConfiguration('end_effector_profile')

    robot_description = OpaqueFunction(function=_build_robot_description, args=[arm_id, load_gripper, franka_hand, load_end_effector_profile, end_effector_profile])
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    set_controller_debug = SetEnvironmentVariable(name='CONTROLLER_DEBUG', value=controller_debug)
    end_effector_profile_server = Node(
        package='compliant_controllers',
        executable='load_endeffector_profile.py',
        name='end_effector_profile_server',
        output='screen',
        arguments=[end_effector_profile],
        condition=IfCondition(load_end_effector_profile),
    )
    gazebo = OpaqueFunction(function=_gazebo_include, args=[world, show_gazebo_gui, controller_debug])
    runtime_nodes = OpaqueFunction(
        function=_build_runtime_nodes,
        args=[controller_name, publish_world_to_base, show_rviz, controller_debug],
    )

    return LaunchDescription(
        declared_args + [
            set_controller_debug,
            end_effector_profile_server,
            gazebo,
            robot_description,
            runtime_nodes,
        ]
    )
