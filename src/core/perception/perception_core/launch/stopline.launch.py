from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='perception',
            executable='stopline_camera',
            name='stopline_camera',
            output='screen'
        ),

        Node(
            package='perception',
            executable='stopline_detection',
            name='stopline_detection',
            output='screen'
        )
    ])