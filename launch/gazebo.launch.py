import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )

    franka_gazebo_conf = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'config',
        'franka_gazebo_controllers.yaml'
    )

    custom_controller_conf = os.path.join(
        get_package_share_directory('compliant_controllers'),
        'config',
        'gazebo_controllers.yaml'
    )

    urdf_xml = robot_description_config.toxml()
    urdf_xml = urdf_xml.replace(franka_gazebo_conf, custom_controller_conf)

    robot_description = {'robot_description': urdf_xml}

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )]


def launch_setup(context: LaunchContext, pkg_ros_gz_sim, world, namespace, show_gazebo_gui):
    # Resolve substitutions now (world is a LaunchConfiguration)
    world_str = context.perform_substitution(world)
    show_gui_str = LaunchConfiguration('show_gazebo_gui').perform(context).lower()
    show_gui = show_gui_str in ('true', '1', 'yes')

    print(show_gui)

    # Build single string arg (gz_sim.launch.py expects a string for gz_args)
    if show_gui:
        gz_args = f"{world_str} -r"
    else:
        gz_args = f"{world_str} -v 4 -s -r --headless-rendering"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return [gazebo]


def generate_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'
    world_name = 'world'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)
    show_rviz = LaunchConfiguration('show_rviz')
    show_gazebo_gui = LaunchConfiguration('show_gazebo_gui')
    world = LaunchConfiguration(world_name)

    # Declare arguments
    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name, default_value='false',
        description='true/false for activating the gripper'
    )
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name, default_value='franka_hand',
        description='Default value: franka_hand'
    )
    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name, default_value='fr3',
        description='Available values: fr3, fp3 and fer'
    )
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name, default_value='',
        description='Namespace for the robot.'
    )
    show_rviz_launch_argument = DeclareLaunchArgument(
        'show_rviz', default_value='true',
        description='Whether to start RViz (default true).'
    )
    show_gazebo_gui_launch_argument = DeclareLaunchArgument(
        'show_gazebo_gui', default_value='false',
        description='Whether to show Gazebo GUI (default false: headless simulation).'
    )
    world_launch_argument = DeclareLaunchArgument(
        world_name, default_value='empty.sdf',
        description='World SDF file (must be in Gazebo resource path).'
    )
    controller_name = LaunchConfiguration('controller')
    controller_launch_argument = DeclareLaunchArgument(
        'controller', default_value='cartesian_impedance_controller',
        description='Name of the controller to load and activate.'
    )

    # Robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand]
    )

    # Gazebo setup
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo = OpaqueFunction(
        function=launch_setup,
        args=[pkg_ros_gz_sim, world, namespace, show_gazebo_gui]
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['--display-config', rviz_file, '-f', 'world'],
        condition=IfCondition(show_rviz),
    )

    # Controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller_name],
        output='screen'
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        show_rviz_launch_argument,
        show_gazebo_gui_launch_argument,
        controller_launch_argument,
        world_launch_argument,
        gazebo,
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{'source_list': ['joint_states'], 'rate': 30}],
        ),
    ])
