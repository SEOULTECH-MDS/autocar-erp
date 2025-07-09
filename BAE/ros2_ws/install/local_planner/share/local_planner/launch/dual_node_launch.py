from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Local Planner Node
        Node(
            package='local_planner',
            executable='local_planner_node',
            name='local_planner_node',
            output='screen'
        ),

        # Dummy Input Node
        Node(
            package='local_planner',
            executable='dummy_input_node',
            name='dummy_input_node',
            output='screen'
        ),

        # Mode Selector Node
        Node(
            package='mode_selector',
            executable='mode_selector_node',  # 이 이름으로 설치돼 있어야 함
            name='mode_selector_node',
            output='screen'
        )
    ])