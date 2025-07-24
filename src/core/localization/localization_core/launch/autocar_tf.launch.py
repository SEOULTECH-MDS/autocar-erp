from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Nodes ---
        # 1. TF Publisher Node
        Node(
            package='localization_core',
            executable='autocar_tf_publisher',
            name='autocar_tf_publisher',
            parameters=[{
                # 'map_origin_lat': 37.632010,  
                # 'map_origin_lon': 127.076008   # 하이테크 뒤

                'map_origin_lat': 37.630117,
                'map_origin_lon': 127.081431   # 미래관 주차장 
            }]
        )
    ]) 