import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch composable nodes."""

    # Declare launch arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/velodyne_points",
            description="Raw input point cloud topic.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "filtered_topic",
            default_value="/velodyne_points/filtered",
            description="Filtered point cloud topic.",
        )
    )
    # Region filter parameters
    declared_args.append(DeclareLaunchArgument("min_x", default_value="-1.8"))
    declared_args.append(DeclareLaunchArgument("max_x", default_value="-0.5"))
    declared_args.append(DeclareLaunchArgument("min_y", default_value="-0.8"))
    declared_args.append(DeclareLaunchArgument("max_y", default_value="0.8"))
    declared_args.append(DeclareLaunchArgument("min_z", default_value="-0.5"))
    declared_args.append(DeclareLaunchArgument("max_z", default_value="0.0"))
    # inside_mode: true => 박스 내부 유지(기본값: 전체 유지 = 무필터), false => 박스 내부 제거
    declared_args.append(DeclareLaunchArgument("inside_mode", default_value="false"))

    # Define composable nodes
    composable_nodes = [
        ComposableNode(
            package='patchworkpp',
            plugin='patchworkpp::PatchworkppPointXYZI',
            name='patchworkpp_node',
            parameters=[{
                'cloud_topic': LaunchConfiguration("filtered_topic"),
                'frame_id': 'velodyne',
                'sensor_height': 0.70,
                'num_iter': 4,
                'num_lpr': 30,
                'num_min_pts': 20,
                'th_seeds': 0.55,
                'th_dist': 0.50,
                'max_range': 80.0,
                'min_range': 0.5,
                'uprightness_thr': 0.92,
                'adaptive_seed_selection_margin': 0.1,
                'verbose': False,
            }],
            remappings=[
                ("nonground", "/patchwork/nonground"),
                ("ground", "/patchwork/ground"),
                ("plane", "/patchwork/plane"),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='adaptive_clustering',
            plugin='perception::AdaptiveClusteringGroundRemoved',
            name='adaptive_clustering_node',
            parameters=[{
                'sensor_model': "VLP-16",
                'cluster_size_min': 10,
                'cluster_size_max': 10000,
                'tolerance_offset': 0.11,
            }],
            remappings=[
                ("velodyne_points_ground_removed", "/patchwork/nonground"),
                ("clusters", "/adaptive_clustering/clusters"),
                ("poses", "/adaptive_clustering/poses"),
                ("markers", "/adaptive_clustering/markers"),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

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
        ],
    )

    # Create the container for the composable nodes
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return LaunchDescription(declared_args + [region_filter_node, container]) 