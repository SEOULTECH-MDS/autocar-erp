from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy',
            executable='cones_node',
            name='cones_node',
            output='screen',
            parameters=[{'open_area_pattern': 2}],
        ),
        Node(
            package='dummy',
            executable='parking_area_node',
            name='parking_area_node',
            output='screen',
        ),
        Node(
            package='dummy',
            executable='initial_pose_node',
            name='initial_pose_node',
            output='screen',
            parameters=[{'publish_current_pose': True, 'rate': 10.0}],
        ),
        Node(
            package='dummy',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{'publish_current_pose': True, 'publish_tf': True}],
        ),
        Node(
            package='planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[
                {'yaw_offset_deg': 30.0},
                {'s_overshoot': 0.6},
                {'clear_lateral': 0.7},
                {'stage2_guided': True},
                {'stage2_k_yaw': 0.0},
            ],
        ),
    ])


