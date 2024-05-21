import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_integration',  # Replace with your package name
            executable='action_server',
            name='action_server',
            output='screen',
        ),
        Node(
            package='action_integration',
            executable='action_client',
            name='action_client',
            output='screen',
        )
    ])