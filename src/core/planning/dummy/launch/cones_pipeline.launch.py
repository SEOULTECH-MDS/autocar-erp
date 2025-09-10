#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    input_markers = LaunchConfiguration('input_markers', default='/sensor_fusion/tracked_rubber_cones')
    output_obstacles = LaunchConfiguration('output_obstacles', default='/cones_obstacles')
    target_frame = LaunchConfiguration('target_frame', default='map')

    pkg_prefix = get_package_prefix('dummy')
    bin_dir = os.path.join(pkg_prefix, 'bin')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('input_markers', default_value='/sensor_fusion/tracked_rubber_cones', description='Rubber cones MarkerArray topic'),
        DeclareLaunchArgument('output_obstacles', default_value='/cones_obstacles', description='Output ObstacleArray topic'),
        DeclareLaunchArgument('target_frame', default_value='map', description='Target frame for map coordinates'),

        # 1) rubber_cones_adapter_node : ros2 run으로 실행
        ExecuteProcess(
            cmd=[
                os.path.join(bin_dir, 'rubber_cones_adapter_node'),
                '--ros-args',
                '-p', ['input_markers:=', input_markers],
                '-p', ['output_obstacles:=', output_obstacles],
                '-p', ['target_frame:=', target_frame],
                '-p', 'enable_side_label:=true',
                '-p', ['use_sim_time:=', use_sim_time],
                '-r', ['/sensor_fusion/tracked_rubber_cones:=', input_markers],
                '-r', ['/cones_obstacles:=', output_obstacles],
            ],
            output='screen',
        ),

        # 2) cone_map_node : ros2 run으로 실행
        ExecuteProcess(
            cmd=[
                os.path.join(bin_dir, 'cone_map_node'),
                '--ros-args',
                '-p', ['input_topic:=', output_obstacles],
                '-p', 'output_topic:=/cone_map',
                '-p', 'visualization_topic:=/cone_map_markers',
                '-p', ['use_sim_time:=', use_sim_time],
                '-r', ['/cones_obstacles:=', output_obstacles],
            ],
            output='screen',
        ),

        # 3) parking_area_node : ros2 run으로 실행
        ExecuteProcess(
            cmd=[
                os.path.join(bin_dir, 'parking_area_node'),
                '--ros-args',
                '-p', ['use_sim_time:=', use_sim_time],
                '-r', ['/cones_obstacles:=', output_obstacles],
                '-r', '/open_parking_area_id:=/open_parking_area_id',
                '-r', '/virtual_walls:=/virtual_walls',
                '-r', '/open_slot_pose:=/open_slot_pose',
                '-r', '/parking_area_visualization:=/parking_area_visualization',
            ],
            output='screen',
        ),
    ])


