from launch import LaunchDescription
from launch_ros.actions import Node

# Assumes the controller is declared inside an already running controller_manager
# You can alternatively load & configure via the spawner utility.

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['cartesian_impedance_controller'],
            output='screen'
        )
    ])
