from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if available'
    )

    map_osm_arg = DeclareLaunchArgument(
        'map_osm_path',
        default_value='src/core/localization/localization_core/data/kcity_qualifying/lanelet2_map.osm',
        description='OSM file path containing parking_path ways'
    )

    path_id_arg = DeclareLaunchArgument(
        'path_id', default_value='1',
        description='Selected parking_path path_id'
    )

    activation_lanelet_id_arg = DeclareLaunchArgument(
        'activation_lanelet_id', default_value='2',
        description='Lanelet ID to trigger qualifying parking'
    )

    reverse_distance_arg = DeclareLaunchArgument(
        'reverse_distance_m', default_value='5.0',
        description='Reverse distance after stopping'
    )

    stop_duration_arg = DeclareLaunchArgument(
        'stop_duration_sec', default_value='3.0',
        description='Stop duration at final forward pose'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_osm_arg,
        path_id_arg,
        activation_lanelet_id_arg,
        reverse_distance_arg,
        stop_duration_arg,

        Node(
            package='planner',
            executable='qualifying_parking_planner',
            name='qualifying_parking_planner',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_osm_path': LaunchConfiguration('map_osm_path'),
                'path_id': LaunchConfiguration('path_id'),
                'activation_lanelet_id': LaunchConfiguration('activation_lanelet_id'),
                'reverse_distance_m': LaunchConfiguration('reverse_distance_m'),
                'stop_duration_sec': LaunchConfiguration('stop_duration_sec'),
            }]
        ),
    ])


