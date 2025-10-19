from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node( # 전방 카메라: 장애물, 배달
           package='perception',
           executable='camera_front',
           name='camera_front',
           output='screen'
        ),

        Node( # 측면 카메라: 주차 라바콘
            package='perception',
            executable='camera_side',
            name='camera_side',
            output='screen'
        ),

        Node( # 뒷쪽 카메라: 신호등
            package='perception',
            executable='camera_trafficlight',
            name='camera_trafficlight',
            output='screen'
        )
    ])