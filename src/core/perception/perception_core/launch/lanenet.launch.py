from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='perception',
            executable='camera_pub',
            name='camera_pub',
            output='screen'
        ),

        Node(
            package='perception',
            executable='lanenet',
            name='lanenet',
            output='screen'
        )
    ])