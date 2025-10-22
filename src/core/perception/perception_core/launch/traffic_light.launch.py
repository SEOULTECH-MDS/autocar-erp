from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='perception',
            executable='camera_trafficlight',
            name='camera_trafficlight',
            output='screen'
        ),

        Node(
            package='perception',
            executable='trafficlight',
            name='trafficlight',
            output='screen'
        )
    ])