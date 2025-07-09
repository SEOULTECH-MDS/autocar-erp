from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='perception',
        executable='stopline_camera',
        name='stopline_camera'
    )

    detection_node = Node(
        package='perception',
        executable='stopline_detection',
        name='stopline_detection'
    )

    return LaunchDescription([
        camera_node,
        detection_node
    ])
