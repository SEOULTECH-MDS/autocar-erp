from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='perception',
        executable='combined_camera',
        name='combined_camera'
    )
    '''
    camera_node = Node(
        package='perception',
        executable='camera_obstacle',
        name='camera_obstacle'
    )
    '''
    detection_node = Node(
        package='perception',
        executable='obstacle',
        name='obstacle'
    )

    return LaunchDescription([
        camera_node,
        detection_node
    ])