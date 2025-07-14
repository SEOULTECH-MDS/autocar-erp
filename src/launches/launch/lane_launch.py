from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='perception',
        executable='camera_pub',
        name='camera_pub'
    )

    detection_node = Node(
        package='perception',
        executable='lanenet',
        name='lanenet'
    )

    return LaunchDescription([
        camera_node,
        detection_node
    ])