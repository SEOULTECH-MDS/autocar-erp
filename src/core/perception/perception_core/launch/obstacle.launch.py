from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        #Node(
        #    package='perception',
        #    executable='camera_car',
        #    name='camera_car',
        #    output='screen'
        #),

        Node( # 라바콘 인식 (드럼통 대신)
            package='perception',
            executable='obstacle',
            name='obstacle',
            output='screen'
        ),

        Node(
            package='perception',
            executable='car_detect',
            name='car_detect',
            output='screen'
        ),

        Node(
            package='perception',
            executable='sensor_fusion_obstacle',
            name='sensor_fusion_obstacle',
            output='screen'
        ),
    ])