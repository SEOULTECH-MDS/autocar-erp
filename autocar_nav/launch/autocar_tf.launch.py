from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autocar_nav',
            executable='autocar_tf_ros2',
            name='autocar_tf',
            parameters=[{
                # 'map_origin_lat': 37.632010,  
                # 'map_origin_lon': 127.076008   # 하이테크 뒤

                'map_origin_lat': 37.630117,
                'map_origin_lon': 127.081431   # 미래관 주차장 
            }]
        )
    ]) 