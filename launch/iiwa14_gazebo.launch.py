"""Gazebo simulation launch for IIWA14 robot using compliant controllers.

Mirrors lbr_fri_ros2_stack's Gazebo launch style for iiwa14 robot.
"""

import os
import xacro
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


def _build_robot_description(context: LaunchContext, arm_id, load_gripper):
    """Build robot description from XACRO, replacing controller config paths."""
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)

    # Path to IIWA14 XACRO file from lbr_description
    iiwa_xacro = os.path.join(
        get_package_share_directory('lbr_description'),
        'urdf', arm_id_str, f'{arm_id_str}.xacro'
    )
    
    # Process XACRO with gazebo parameters - enable effort control
    xacro_doc = xacro.process_file(
        iiwa_xacro,
        mappings={
            'robot_name': arm_id_str,
            'gripper': load_gripper_str,
            'mode': 'gazebo',
            'gazebo_effort': 'true',  # Enable effort interface for cartesian controller
        }
    )
    
    # Get default LBR controller config
    default_yaml = os.path.join(
        get_package_share_directory('lbr_description'),
        'ros2_control', 'gazebo_controllers.yaml'
    )
    
    # Use custom config from compliant_controllers if available, else use default
    try:
        custom_yaml = os.path.join(
            get_package_share_directory('compliant_controllers'),
            'config', 'lbr_gz_controllers.yaml'
        )
        if os.path.exists(custom_yaml):
            urdf_xml = xacro_doc.toxml().replace(default_yaml, custom_yaml)
        else:
            urdf_xml = xacro_doc.toxml()
    except:
        urdf_xml = xacro_doc.toxml()
    
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=arm_id_str,
        output='both',
        parameters=[{'robot_description': urdf_xml, 'use_sim_time': True}]
    )]


def _gazebo_include(context: LaunchContext, world, show_gazebo_gui, controller_debug):
    """Include Gazebo launch with appropriate arguments."""
    world_file = context.perform_substitution(world)
    gui_flag = context.perform_substitution(show_gazebo_gui).lower() in ('true', '1', 'yes')
    debug_flag = context.perform_substitution(controller_debug).lower() in ('true', '1', 'yes')

    if gui_flag:
        gz_args = f"{world_file} -r"
    else:
        gz_args = f"{world_file} -v 4 -s -r --headless-rendering"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return [gazebo_launch]


def _build_runtime_nodes(context: LaunchContext, controller_name, publish_world_to_base, show_rviz, arm_id):
    """Build runtime nodes for spawning and controller management."""
    controller = context.perform_substitution(controller_name)
    rviz_flag = context.perform_substitution(show_rviz).lower() in ('true', '1', 'yes')
    arm_id_str = context.perform_substitution(arm_id)
    controller_manager = f'/{arm_id_str}/controller_manager'

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_iiwa14',
        arguments=['-topic', 'robot_description', '-name', arm_id_str, '-allow_renaming'],
        output='screen',
        namespace=arm_id_str
    )
    
    # Joint state broadcaster spawner args
    controller_config = os.path.join(
        get_package_share_directory('compliant_controllers'),
            'config', 'lbr_gz_controllers.yaml'
    )
    jsb_args = ['joint_state_broadcaster', '--controller-manager', 'controller_manager', '--params-file', controller_config]

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=jsb_args,
        namespace=arm_id_str
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('compliant_controllers'),
                'launch',
                'generic_controller_wrapper.launch.py',
            )
        ),
        launch_arguments={
            'namespace': arm_id_str,
            'arm_id': 'lbr',
            'controller_name': controller,
            'controller_manager': controller_manager,
            'impl_library': LaunchConfiguration('impl_library'),
            'init_k_pos': LaunchConfiguration('init_k_pos'),
            'init_k_ori': LaunchConfiguration('init_k_ori'),
            'joints': 'lbr_A1,lbr_A2,lbr_A3,lbr_A4,lbr_A5,lbr_A6,lbr_A7',
            'ee_frame': 'lbr_link_ee',
            'base_frame': 'lbr_link_0',
            'robot_description_node': f'/{arm_id_str}/robot_state_publisher',
            'robot_description_param': 'robot_description',
            'load_end_effector_profile': 'false',
            'add_gravity_compensation': LaunchConfiguration('add_gravity_compensation'),
            'compensate_end_effector_load': LaunchConfiguration('compensate_end_effector_load'),
            'add_friction_compensation': LaunchConfiguration('add_friction_compensation'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'diagnostic_log_file': LaunchConfiguration('diagnostic_log_file'),
            'diagnostic_log_duration': LaunchConfiguration('diagnostic_log_duration'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
        }.items(),
    )

    # Chain events: spawn -> joint_state_broadcaster -> main controller wrapper.
    spawn_to_js = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner]))
    js_to_main = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[include_controller]))

    nodes = [spawn, spawn_to_js, js_to_main]

    # RViz visualization
    if rviz_flag:
        rviz_config = os.path.join(
            get_package_share_directory('lbr_bringup'),
            'config', 'gazebo.rviz'
        )
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['--display-config', rviz_config, '-f', 'world']
        ))

    # Joint state publisher
    nodes.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['joint_states'], 'rate': 30}],
        output='screen'
    ))

    return nodes


def generate_launch_description():
    """Generate launch description for IIWA14 Gazebo simulation."""
    declared_args = [
        DeclareLaunchArgument(
            'arm_id',
            default_value='iiwa14',
            description='Robot model identifier (iiwa14, iiwa7, med7, med14)'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace (unused in simulation)'
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
            description='Load gripper in URDF'
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='cartesian_impedance_controller',
            description='Primary controller to spawn (effort-based: cartesian_impedance_controller, lbr_joint_impedance_controller, etc.)'
        ),
        DeclareLaunchArgument(
            'impl_library',
            default_value='libcartesian_impedance_impl.so',
            description='Generic wrapper implementation library'
        ),
        DeclareLaunchArgument('init_k_pos', default_value='200.0'),
        DeclareLaunchArgument('init_k_ori', default_value='10.0'),
        DeclareLaunchArgument('add_gravity_compensation', default_value='true'),
        DeclareLaunchArgument('compensate_end_effector_load', default_value='false'),
        DeclareLaunchArgument('add_friction_compensation', default_value='false'),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
        DeclareLaunchArgument('diagnostic_log_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_duration', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_mode', default_value='0'),
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'show_gazebo_gui',
            default_value='false',
            description='Show Gazebo GUI (requires X11 forwarding for headless systems)'
        ),
        DeclareLaunchArgument(
            'show_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'publish_world_to_base',
            default_value='true',
            description='Publish static world->base transform'
        ),
        DeclareLaunchArgument(
            'controller_debug',
            default_value='false',
            description='Enable debug logging for controller_manager'
        ),
    ]

    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    world = LaunchConfiguration('world')
    show_gazebo_gui = LaunchConfiguration('show_gazebo_gui')
    controller_debug = LaunchConfiguration('controller_debug')
    controller_name = LaunchConfiguration('controller_name')
    publish_world_to_base = LaunchConfiguration('publish_world_to_base')
    show_rviz = LaunchConfiguration('show_rviz')

    # OpaqueFunction calls to build descriptions at launch time with context
    robot_description = OpaqueFunction(
        function=_build_robot_description,
        args=[arm_id, load_gripper]
    )
    
    # Set GZ_SIM_RESOURCE_PATH to find robot models
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('lbr_description')
    )
    
    set_controller_debug = SetEnvironmentVariable(
        name='CONTROLLER_DEBUG',
        value=controller_debug
    )
    
    gazebo = OpaqueFunction(
        function=_gazebo_include,
        args=[world, show_gazebo_gui, controller_debug]
    )
    
    runtime_nodes = OpaqueFunction(
        function=_build_runtime_nodes,
        args=[controller_name, publish_world_to_base, show_rviz, arm_id]
    )

    return LaunchDescription(declared_args + [
        set_controller_debug,
        gazebo,
        robot_description,
        runtime_nodes
    ])
