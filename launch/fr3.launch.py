# Launch real Franka Robotics FR3 robot with compliant controllers
# Uses franka.launch.py from franka_bringup package and spawns
# compliant controllers defined in fr3_controllers.yaml (inside this package).

# By defult, a dummy static transform from world to robot base is published.
# It can be disabled by setting publish_world_to_base:=false

# For visualization, RViz can be optionally launched using use_rviz:=true flag

# For other parameter descriptions, see franka.launch.py documentation

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments (exposed parameters of franka.launch.py) with inline defaults
    declared_args = [
        DeclareLaunchArgument('arm_id', default_value='fr3',
                              description='Unique arm identifier'),
        DeclareLaunchArgument('arm_prefix', default_value='',
                              description='Arm prefix (reserved for future use)'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS namespace for the robot (empty = no namespace)'),
        DeclareLaunchArgument('urdf_file', default_value='fr3/fr3.urdf.xacro',
                              description='Relative path to URDF in franka_description/robots'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.1',
                              description='Robot IP address or hostname'),
        DeclareLaunchArgument('load_gripper', default_value='false',
                              description='Whether to load the Franka gripper'),
        DeclareLaunchArgument('joint_state_rate', default_value='30',
                              description='Joint state publish rate (Hz)'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller',
                              description='Primary controller to spawn'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz for visualization'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true',
                              description='Publish static transform from world to robot base'),
    ]

    include_franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])
        ),
        launch_arguments={
            'arm_id': LaunchConfiguration('arm_id'),
            'arm_prefix': LaunchConfiguration('arm_prefix'),
            'namespace': LaunchConfiguration('namespace'),
            'urdf_file': LaunchConfiguration('urdf_file'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'controllers_yaml': PathJoinSubstitution([
                FindPackageShare('compliant_controllers'), 'config', 'fr3_controllers.yaml'
            ]),
        }.items(),
    )

    # Controller spawner using this package's fr3_controllers.yaml
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=[LaunchConfiguration('controller_name'), '--controller-manager-timeout', '30'],
        output='screen',
    )

    # Optional static transform that anchors the robot's base frame to the world frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world',
                   PathJoinSubstitution([LaunchConfiguration('namespace'), 'base'])],
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_world_to_base')),
    )

    # Optional RViz instance
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', PathJoinSubstitution([
            FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    nodes = [spawner, static_tf_node, rviz_node]

    return LaunchDescription(declared_args + [include_franka] + nodes)
