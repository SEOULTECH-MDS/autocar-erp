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

        # Lane Mission Controller (미션별 활성화 제어)
        Node(
            package='perception',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'config_file': 'kcity_config.yaml'}
            ]
        ),

        # 신호등 탐지
        Node(
            package='perception',
            executable='trafficlight',
            name='trafficlight',
            output='screen'
        ),

        # 배달 표지판 탐지
        Node(
            package='perception',
            executable='sign',
            name='sign',
            output='screen'
        ),
        Node(
            package='perception',
            executable='sensor_fusion_sign',
            name='sensor_fusion_sign',
            output='screen'
        ),

        # 장애물 탐지
        Node( # 드럼통 인식
            package='perception',
            executable='drum_detect',
            name='drum_detect',
            output='screen'
        ),
        Node( # 차 인식
            package='perception',
            executable='car_detect',
            name='car_detect',
            output='screen'
        ),
        Node(
            package='perception',
            executable='sensor_fusion_obstacle',
            name='sensor_fusion_obstacle',
            output='screen'
        ),

        # 주차 라바콘 탐지
        Node(
            package='perception',
            executable='rubber_detect',
            name='rubber_detect',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='perception',
            executable='sensor_fusion_rubber',
            name='sensor_fusion_rubber',
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