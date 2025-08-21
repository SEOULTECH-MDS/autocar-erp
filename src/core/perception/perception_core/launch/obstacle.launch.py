from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Node(
        #     package='perception',
        #     executable='left_camera',
        #     name='left_camera',
        #     output='screen'
        # ),

        # Node(
        #     package='perception',
        #     executable='right_camera',
        #     name='right_camera',
        #     output='screen'
        # ),

        # Node(
        #     package='perception',
        #     executable='combined_camera',
        #     name='combined_camera',
        #     output='screen'
        # ),

        Node(
            package='perception',
            executable='obstacle',
            name='obstacle',
            output='screen'
        ),

        Node(
            package='perception',
            executable='sensor_fusion_object',
            name='sensor_fusion_object',
            output='screen'
        ),

        Node(
            package='perception',
            executable='bbox_tracker',
            name='bbox_tracker',
            output='screen'
        ),

        Node(
            package='perception',
            executable='object_tracker3D',
            name='object_tracker3D',
            output='screen'
        ),

        Node(
            package='perception',
            executable='rubber_visualizer',
            name='rubber_visualizer',
            output='screen'
        )
    ])