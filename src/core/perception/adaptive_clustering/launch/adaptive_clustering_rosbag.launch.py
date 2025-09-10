from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Common args
    declare_sensor_model_arg = DeclareLaunchArgument(
        'sensor_model', default_value='VLP-16',
        description='The sensor model to use.'
    )
    declare_print_fps_arg = DeclareLaunchArgument(
        'print_fps', default_value='true',
        description='Whether to print FPS information.'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if available.'
    )

    # Restamp topics
    declare_input_cloud_arg = DeclareLaunchArgument(
        'bag_cloud_topic', default_value='/velodyne_points',
        description='Bag source PointCloud2 topic.'
    )
    declare_restamped_topic_arg = DeclareLaunchArgument(
        'restamped_topic', default_value='/velodyne_points/restamped',
        description='Restamped PointCloud2 topic.'
    )

    # Region filter args (same as original)
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

    # 0) Restamp node
    restamp_node = Node(
        package='adaptive_clustering',
        executable='restamp_pointcloud.py',
        name='restamp_pointcloud',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('bag_cloud_topic'),
            'output_topic': LaunchConfiguration('restamped_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 1) Region filter
    region_filter_node = Node(
        package='perception',
        executable='region_filter_node',
        name='region_filter_node',
        output='screen',
        parameters=[
            {'input_topic': LaunchConfiguration('restamped_topic')},
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

    # 2) Adaptive clustering container
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
                    'sensor_model': LaunchConfiguration('sensor_model'),
                    'print_fps': LaunchConfiguration('print_fps'),
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
        declare_use_sim_time_arg,
        declare_input_cloud_arg,
        declare_restamped_topic_arg,
        declare_filtered_topic_arg,
        declare_min_x_arg,
        declare_max_x_arg,
        declare_min_y_arg,
        declare_max_y_arg,
        declare_min_z_arg,
        declare_max_z_arg,
        declare_inside_mode_arg,
        restamp_node,
        region_filter_node,
        container,
    ])


