from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autocar_nav',
            executable='autocar_tf_ros2',
            name='autocar_tf',
            parameters=[{
                'map_origin_lat': 37.2386,  # 원하는 위도 값
                'map_origin_lon': 126.7728   # 원하는 경도 값
            }]
        )
    ]) 