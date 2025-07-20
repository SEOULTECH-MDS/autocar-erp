#!/usr/bin/env python3
"""
Simple Test Launch - 간단한 테스트 실행 파일

이 파일은 센서 데이터 시뮬레이션과 모드 셀렉터만 실행합니다.

실행하는 노드들:
1. tf_broadcaster: 좌표계 변환 브로드캐스터
2. mode_selector_simple: 간단한 모드 셀렉터
3. simple_test: 센서 데이터 시뮬레이션

주요 기능:
- 기본적인 모드 셀렉터 테스트 환경 제공
- 시각화 없이 콘솔 출력만으로 테스트
- 빠른 테스트를 위한 최소 구성

사용법:
    ros2 launch path_planning simple_test_launch.py

참고:
    이 launch 파일은 시각화 없이 모드 셀렉터의 로직만 테스트할 때 사용합니다.
    콘솔에서 로그를 확인하여 동작을 검증할 수 있습니다.
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
            default_value='false',
            description='Use RViz for visualization'
        ),
        
        # Mode Selector Simple Node
        Node(
            package='path_planning',
            executable='mode_selector_simple',
            name='mode_selector_simple',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Simple Test Node
        Node(
            package='path_planning',
            executable='simple_test',
            name='simple_test',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ]) 