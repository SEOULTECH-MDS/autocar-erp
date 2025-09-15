from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if available'
    )

    return LaunchDescription([
        use_sim_time_arg,
        #Node(
        #    package='perception',
        #    executable='camera_rubber',
        #    name='camera_rubber',
        #    output='screen'
        #),

        Node(
            package='perception',
            executable='rubber_detect',
            name='rubber_detect',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package='perception',
            executable='sensor_fusion_object',
            name='sensor_fusion_object',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/markers', '/adaptive_clustering/markers')],
        ),

        Node(
            package='perception',
            executable='bbox_tracker',
            name='bbox_tracker',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package='perception',
            executable='object_tracker3D',
            name='object_tracker3D',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package='perception',
            executable='rubber_visualizer',
            name='rubber_visualizer',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])