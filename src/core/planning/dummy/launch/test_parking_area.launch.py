#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Package directory
    pkg_dummy = get_package_share_directory('dummy')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Cones Node (라바콘 생성)
        Node(
            package='dummy',
            executable='cones_node',
            name='cones_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/cones_markers', '/cones_markers'),
                ('/cones_obstacles', '/cones_obstacles'),
            ]
        ),
        
        # Parking Area Node (주차 구역 분석)
        Node(
            package='dummy',
            executable='parking_area_node',
            name='parking_area_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # 주차 구역 기준점 (라바콘 위치에 따라 조정)
                'slot_origin_x': 0.0,      # X 기준선 (도로 가장자리)
                'slot_origin_y': 0.0,      # 가장 아래 슬롯의 y 하단
                'SLOT_LEN': 5.0,           # L : 세로 길이
                'SLOT_GAP': 0.01,          # G : 슬롯 간 간격
                'SLOT_WIDTH': 2.5,         # W : 가로 폭
                'EPS_X': 0.3,              # 좌/우 여유 허용치
                'EPS_Y': 0.01,             # 상/하 여유 허용치
            }],
            remappings=[
                ('/cones_obstacles', '/cones_obstacles'),
                ('/open_parking_area_id', '/open_parking_area_id'),
                ('/virtual_walls', '/virtual_walls'),
                ('/open_slot_pose', '/open_slot_pose'),
                ('/parking_area_visualization', '/parking_area_visualization'),
            ]
        ),
    ])
