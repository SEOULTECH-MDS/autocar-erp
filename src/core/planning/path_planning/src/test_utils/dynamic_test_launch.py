#!/usr/bin/env python3
"""
Dynamic Test Launch - 동적 테스트 실행 파일

이 파일은 동적 차량 시뮬레이션을 포함한 고급 테스트를 실행합니다.

실행하는 노드들:
1. tf_broadcaster: 좌표계 변환 브로드캐스터
2. mode_selector_simple: 간단한 모드 셀렉터
3. dynamic_vehicle_test: 동적 차량 시뮬레이션
4. enhanced_visualizer: 고급 시각화
5. rviz2: 3D 시각화 도구

주요 기능:
- 움직이는 차량을 시뮬레이션한 현실적인 테스트
- 고급 시각화 기능 제공
- 동적 센서 데이터 생성

사용법:
    ros2 launch path_planning dynamic_test_launch.py

참고:
    이 launch 파일은 가장 현실적인 테스트 환경을 제공합니다.
    차량이 실제로 움직이는 상황을 시뮬레이션합니다.
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
        
        # Dynamic Vehicle Test Node (차량 위치 시뮬레이션)
        Node(
            package='path_planning',
            executable='dynamic_vehicle_test',
            name='dynamic_vehicle_test',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Enhanced Visualizer Node (개선된 시각화)
        Node(
            package='path_planning',
            executable='enhanced_visualizer',
            name='enhanced_visualizer',
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
                'dynamic_test.rviz'
            )],
            condition=LaunchConfiguration('use_rviz'),
            output='screen'
        ),
    ]) 