#!/usr/bin/env python3
"""
Visualization Launch - 시각화 전용 실행 파일

이 파일은 모드 셀렉터의 시각화 관련 노드들만 실행합니다.

실행하는 노드들:
1. tf_broadcaster: 좌표계 변환 브로드캐스터
2. mode_selector_visualizer: RViz 시각화
3. rviz2: 3D 시각화 도구

주요 기능:
- 시각화에 필요한 노드들만 실행
- 모드 셀렉터나 테스트 노드는 실행하지 않음
- 기존에 실행 중인 노드들과 함께 사용 가능

사용법:
    ros2 launch path_planning visualization_launch.py

참고:
    이 launch 파일은 이미 실행 중인 모드 셀렉터나 테스트 노드가 있을 때
    시각화만 추가로 실행하고 싶을 때 사용합니다.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('path_planning'),
            'src/mode_selector_test/mode_selector.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Find package share directory
    pkg_share = FindPackageShare('path_planning')
    
    # Mode selector node
    mode_selector_node = Node(
        package='path_planning',
        executable='mode_selector_simple',
        name='mode_selector_simple',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # Test node for sensor simulation
    test_node = Node(
        package='path_planning',
        executable='simple_test',
        name='simple_test_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # Visualization node
    visualizer_node = Node(
        package='path_planning',
        executable='mode_selector_visualizer',
        name='mode_selector_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=LaunchConfiguration('use_rviz'),
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        mode_selector_node,
        test_node,
        visualizer_node,
        rviz_node
    ]) 