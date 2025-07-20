#!/usr/bin/env python3
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