from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
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
    # Point cloud filter args (borrowed from ground_removal_clustering)
    declare_cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/velodyne_points',
        description='Raw input point cloud topic.'
    )
    declare_filtered_topic_arg = DeclareLaunchArgument(
        'filtered_topic', default_value='/velodyne_points/filtered',
        description='Filtered point cloud topic.'
    )
    declare_min_x_arg = DeclareLaunchArgument('min_x', default_value='-1.8')
    declare_max_x_arg = DeclareLaunchArgument('max_x', default_value='-0.5')
    declare_min_y_arg = DeclareLaunchArgument('min_y', default_value='-0.8')
    declare_max_y_arg = DeclareLaunchArgument('max_y', default_value='0.8')
    declare_min_z_arg = DeclareLaunchArgument('min_z', default_value='-0.5')
    declare_max_z_arg = DeclareLaunchArgument('max_z', default_value='0.0')
    declare_inside_mode_arg = DeclareLaunchArgument('inside_mode', default_value='false')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if available.'
    )

    # Get launch configuration
    sensor_model = LaunchConfiguration('sensor_model')
    print_fps = LaunchConfiguration('print_fps')

    # Region filter node (standalone)
    region_filter_node = Node(
        package='perception',
        executable='region_filter_node',
        name='region_filter_node',
        output='screen',
        parameters=[
            {'input_topic': LaunchConfiguration('cloud_topic')},
            {'output_topic': LaunchConfiguration('filtered_topic')},
            {'min_x': LaunchConfiguration('min_x')},
            {'max_x': LaunchConfiguration('max_x')},
            {'min_y': LaunchConfiguration('min_y')},
            {'max_y': LaunchConfiguration('max_y')},
            {'min_z': LaunchConfiguration('min_z')},
            {'max_z': LaunchConfiguration('max_z')},
            {'inside_mode': LaunchConfiguration('inside_mode')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

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
                        'cluster_size_min': 15,
                        'x_axis_min': -1.0,
                        'x_axis_max': 15.0,
                        'y_axis_min': -6.0,
                        'y_axis_max': 6.0,
                        'z_axis_min': -0.60,
                        'z_axis_max': 0.5,
                        'tolerance_offset': 0.15,
                        'use_sim_time': LaunchConfiguration('use_sim_time')
                    }],
                    remappings=[
                        ("velodyne_points", LaunchConfiguration('filtered_topic')),
                        ("clusters", "/adaptive_clustering/clusters"),
                        ("poses", "/adaptive_clustering/poses"),
                        ("markers", "/adaptive_clustering/markers"),
                    ],
                ),
            ],
            output='screen',
    )

    return LaunchDescription([
        declare_sensor_model_arg,
        declare_print_fps_arg,
        declare_cloud_topic_arg,
        declare_filtered_topic_arg,
        declare_min_x_arg,
        declare_max_x_arg,
        declare_min_y_arg,
        declare_max_y_arg,
        declare_min_z_arg,
        declare_max_z_arg,
        declare_inside_mode_arg,
        declare_use_sim_time_arg,
        region_filter_node,
        container
    ]) 