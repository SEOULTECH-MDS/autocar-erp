#!/usr/bin/env python3
"""
Complete Visualization Launch - 통합 시각화 실행 파일

이 파일은 모드 셀렉터 테스트를 위한 모든 노드와 RViz를 한 번에 실행합니다.

실행하는 노드들:
1. tf_broadcaster: 좌표계 변환 브로드캐스터
2. mode_selector_simple: 간단한 모드 셀렉터
3. simple_test: 센서 데이터 시뮬레이션
4. mode_selector_visualizer: RViz 시각화
5. rviz2: 3D 시각화 도구

주요 기능:
- 모든 필요한 노드를 순서대로 시작
- RViz 설정 파일 자동 로드
- 테스트 환경 완전 자동화

사용법:
    ros2 launch path_planning complete_visualization_launch.py

또는:
    ./run_visualization.sh

참고:
    이 launch 파일은 테스트 환경에서만 사용합니다.
    실제 운영 환경에서는 개별 노드들을 직접 실행합니다.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    # TF Broadcaster (필수 - map 프레임 제공)
    tf_broadcaster_node = Node(
        package='path_planning',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen'
    )
    
    # Mode selector node
    mode_selector_node = Node(
        package='path_planning',
        executable='mode_selector_simple',
        name='mode_selector_simple',
        output='screen'
    )
    
    # Test node for sensor simulation
    test_node = Node(
        package='path_planning',
        executable='simple_test',
        name='simple_test_node',
        output='screen'
    )
    
    # Visualization node
    visualizer_node = Node(
        package='path_planning',
        executable='mode_selector_visualizer',
        name='mode_selector_visualizer',
        output='screen'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=LaunchConfiguration('use_rviz')
    )
    
    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        tf_broadcaster_node,
        mode_selector_node,
        test_node,
        visualizer_node,
        rviz_node
    ]) 