from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autocar_nav',
            executable='path_selector',
            name='path_selector',
            output='screen'
        )
    ]) 