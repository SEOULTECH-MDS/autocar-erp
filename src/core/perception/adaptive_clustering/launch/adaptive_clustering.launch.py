from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with a component container."""
    
    # Declare launch arguments
    declare_sensor_model_arg = DeclareLaunchArgument(
        'sensor_model', default_value='VLP-16',
        description='The sensor model to use.'
    )
    declare_print_fps_arg = DeclareLaunchArgument(
        'print_fps', default_value='true',
        description='Whether to print FPS information.'
    )

    # Get launch configuration
    sensor_model = LaunchConfiguration('sensor_model')
    print_fps = LaunchConfiguration('print_fps')

    # Create a container to host the component
    container = ComposableNodeContainer(
            name='adaptive_clustering_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='adaptive_clustering',
                    plugin='perception::AdaptiveClusteringNode',
                    name='adaptive_clustering',
                    parameters=[{
                        'sensor_model': sensor_model,
                        'print_fps': print_fps,
                        'cluster_size_max': 1000,
                        'x_axis_min': -3.0,
                        'x_axis_max': 7.0,
                        'y_axis_min': -4.0,
                        'y_axis_max': 4.0,
                        'z_axis_min': -0.60,
                        'z_axis_max': 0.5,
                        'tolerance_offset': 0.11
                    }],
                ),
            ],
            output='screen',
    )

    return LaunchDescription([
        declare_sensor_model_arg,
        declare_print_fps_arg,
        container
    ]) 