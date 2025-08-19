#!/usr/bin/env python3
"""
Mode Selector Test Launch - 모드 셀렉터 테스트 실행 파일

이 파일은 모드 셀렉터의 고급 테스트 기능을 실행합니다.

실행하는 노드들:
1. mode_selector_simple: 간단한 모드 셀렉터
2. simple_test: 센서 데이터 시뮬레이션
3. mode_selector_test: 고급 테스트 노드

주요 기능:
- 기본 테스트 + 고급 테스트 기능
- 성능 측정 및 통계 수집
- 자동화된 테스트 실행
- 상세한 테스트 결과 분석

사용법:
    ros2 launch path_planning mode_selector_test_launch.py

참고:
    이 launch 파일은 모드 셀렉터의 성능과 안정성을 종합적으로 테스트합니다.
    개발 중인 고급 테스트 기능들을 포함합니다.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Use RViz for visualization'
        ),
        
        # Mode Selector Node
        Node(
            package='path_planning',
            executable='mode_selector',
            name='mode_selector',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Mode Selector Test Node (센서 데이터 시뮬레이션)
        Node(
            package='path_planning',
            executable='mode_selector_test',
            name='mode_selector_test',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Mode Selector Visualizer Node (RViz 시각화)
        Node(
            package='path_planning',
            executable='mode_selector_visualizer',
            name='mode_selector_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # RViz2 (시각화)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__),
                'mode_selector_test.rviz'
            )],
            condition=LaunchConfiguration('use_rviz'),
            output='screen'
        ),
    ]) 