from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package Paths
    localization_core_share_dir = get_package_share_directory('localization_core')
    
    # Define paths for sample_map test
    default_map_path = os.path.join(localization_core_share_dir, 'data', 'sample_map', 'lanelet2_map.osm')
    
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument(
            'lanelet2_map_path',
            default_value=default_map_path,
            description='Path to the lanelet2 map file to test'
        ),

        # --- Nodes ---
        # 1. Static TF Publisher to align 'world' and 'map' frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # 2. Map Loader Node
        Node(
            package='autoware_map_loader',
            executable='autoware_lanelet2_map_loader',
            name='lanelet2_map_loader',
            parameters=[{'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path')}]
        ),
        
        # 3. RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--fixed-frame', 'world'],
            output='screen'
        )
    ]) 