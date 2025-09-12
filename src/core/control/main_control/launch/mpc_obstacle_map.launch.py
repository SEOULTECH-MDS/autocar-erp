#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
        
        # Obstacle Map Node - 센서 퓨전 데이터를 map 좌표계로 변환
        Node(
            package='main_control',
            executable='obstacle_map_node',
            name='obstacle_map_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'velodyne_x_offset': -1.3,    # velodyne -> base_link 오프셋
                'merge_radius': 0.2,          # 장애물 병합 반경 (m)
                'smoothing_alpha': 0.2,       # EMA 평활화 계수
                'max_age_sec': 10.0,          # 장애물 만료 시간 (초)
                'marker_scale': 0.3           # 마커 크기
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],            
        ),
        
        # MPC Control Node - 변환된 장애물 데이터를 사용하여 차량 제어
        Node(
            package='main_control',
            executable='mpc_acados_control_sp',
            name='mpc_control_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        )
    ])