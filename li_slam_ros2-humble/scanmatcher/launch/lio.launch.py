import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            get_package_share_directory('scanmatcher'),
            'param',
            'lio.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[mapping_param_dir],
        remappings=[('/input_cloud','/cloud_deskewed')],
        output='screen'
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[mapping_param_dir],
        output='screen'
    )

    # Velodyne LiDAR transform: 차량 후륜 중심축으로부터 앞으로 120cm, 지면에서 70cm
    tf_lidar = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1.2','0','0.7','0','0','0','1','base_link','velodyne']
    )

    # IMU transform: 차량 후륜 중심축으로부터 앞으로 30cm, 지면에서 145cm
    tf_imu = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.3','0','1.45','0','0','0','1','base_link','imu_link']
    )

    imu_pre = launch_ros.actions.Node(
        package='scanmatcher',
        executable='imu_preintegration',
        parameters=[mapping_param_dir, {
            'imu_topic': '/imu/data',
            'max_time_diff': 0.05,
            'min_imu_rate': 100.0
        }],
        remappings=[('/odometry','/odom')],
        output='screen'
    )

    img_pro = launch_ros.actions.Node(
        package='scanmatcher',
        executable='image_projection',
        parameters=[mapping_param_dir],
        remappings=[
            ('/points_raw', '/velodyne_points'),  # 직접 velodyne_points 사용
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        mapping,
        tf_lidar,        # LiDAR transform
        tf_imu,          # IMU transform
        imu_pre,
        img_pro,
        graphbasedslam,
    ])