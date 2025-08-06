from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='perception',
            executable='camera_sign',
            name='camera_sign',
            output='screen'
        ),

        Node(
            package='perception',
            executable='sign',
            name='sign',
            output='screen'
        ),

        Node(
            package='perception',
            executable='sensor_fusion_sign',
            name='sensor_fusion_sign',
            output='screen'
        ),
    ])