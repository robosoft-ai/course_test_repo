from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetbot_control_pkg',
            executable='control_node',
            output='screen'),
    ])
