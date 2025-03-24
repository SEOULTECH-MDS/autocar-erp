from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='perception',
        executable='camera_traffic',
        name='camera_traffic'
    )

    detection_node = Node(
        package='perception',
        executable='trafficlight',
        name='trafficlight'
    )

    return LaunchDescription([
        camera_node,
        detection_node
    ])