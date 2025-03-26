from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='perception',
        executable='camera_sign',
        name='camera_sign'
    )

    detection_node = Node(
        package='perception',
        executable='sign',
        name='sign'
    )

    return LaunchDescription([
        camera_node,
        detection_node
    ])