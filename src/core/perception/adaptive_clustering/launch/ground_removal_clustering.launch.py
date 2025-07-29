import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch composable nodes."""

    # Declare launch arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/velodyne_points",
            description="Input point cloud topic.",
        )
    )

    # Define composable nodes
    composable_nodes = [
        ComposableNode(
            package='patchworkpp',
            plugin='patchworkpp::PatchworkppPointXYZI',
            name='patchworkpp_node',
            parameters=[{
                'cloud_topic': LaunchConfiguration("cloud_topic"),
                'frame_id': 'velodyne',
                'sensor_height': 0.42,
                'num_iter': 3,
                'num_lpr': 20,
                'num_min_pts': 10,
                'th_seeds': 0.3,
                'th_dist': 0.125,
                'max_r': 80.0,
                'min_r': 1.0,
                'uprightness_thr': 0.707,
                'verbose': False,
            }],
            remappings=[
                ("nonground", "/patchwork/nonground"),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='adaptive_clustering',
            plugin='perception::AdaptiveClusteringGroundRemoved',
            name='adaptive_clustering_node',
            parameters=[{
                'sensor_model': "VLP-16",
                'cluster_size_min': 3,
                'cluster_size_max': 10000,
                'tolerance_offset': 0.1,
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

    # Create the container for the composable nodes
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return LaunchDescription(declared_args + [container]) 