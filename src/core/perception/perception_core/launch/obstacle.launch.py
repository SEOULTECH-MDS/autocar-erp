from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Node(
        #    package='perception',
        #    executable='camera_front',
        #    name='camera_front',
        #    output='screen'
        # ),

        # Node( # 사람 인식
        #     package='perception',
        #     executable='person_detect',
        #     name='person_detect',
        #     output='screen'
        # ),

        Node( # 드럼통 인식
            package='perception',
            executable='drum_detect',
            name='drum_detect',
            output='screen'
        ),

        # Node( # 차 인식
        #     package='perception',
        #     executable='car_detect',
        #     name='car_detect',
        #     output='screen'
        # ),

        Node(
            package='perception',
            executable='sensor_fusion_obstacle',
            name='sensor_fusion_obstacle',
            output='screen'
        ),
    ])