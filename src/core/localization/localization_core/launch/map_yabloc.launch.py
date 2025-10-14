from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package share directory
    erp42_visualizer_share_dir = get_package_share_directory('erp42_visualizer')

    # --- Launch Arguments ---
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='kcity_v6',
        description='Name of the map folder in localization_core/data'
    )
    map_osm_file_arg = DeclareLaunchArgument(
        'map_osm_file',
        default_value='lanelet2_map.osm',
        description='Name of the .osm file in the map folder'
    )
    map_origin_lat_arg = DeclareLaunchArgument(
        'map_origin_lat',
        default_value='37.239205',  # kcity
        description='Latitude of map origin for UTM projection'
    )
    map_origin_lon_arg = DeclareLaunchArgument(
        'map_origin_lon',
        default_value='126.773193',  # kcity
        description='Longitude of map origin for UTM projection'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if available'
    )

    # Camera topics for YabLoc
    camera_image_topic_arg = DeclareLaunchArgument(
        'camera_image_topic',
        default_value='/usb_cam_1/image_raw',
        description='Input camera image topic for YabLoc'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/usb_cam_1/camera_info',
        description='Input camera info topic for YabLoc'
    )
    # GNSS pose topic supplied by gnss_compass
    gnss_pose_topic_arg = DeclareLaunchArgument(
        'gnss_pose_topic',
        default_value='/sensing/gnss/pose_with_covariance',
        description='GNSS PoseWithCovariance topic used by YabLoc GNSS corrector'
    )
    fix_velocity_topic_arg = DeclareLaunchArgument(
        'fix_velocity_topic',
        default_value='/ublox_gps_node/fix_velocity',
        description='u-blox GNSS fix velocity (TwistWithCovarianceStamped) topic'
    )
    yaw_rate_sign_arg = DeclareLaunchArgument(
        'yaw_rate_sign',
        default_value='-1.0',
        description='IMU yaw rate sign (1.0 or -1.0)'
    )

    # Optional gnss_compass input topics
    gnss_main_fix_arg = DeclareLaunchArgument(
        'gnss_main_fix_topic',
        default_value='/ublox_gps_node/fix',
        description='Main GNSS NavSatFix topic'
    )
    gnss_sub_fix_arg = DeclareLaunchArgument(
        'gnss_sub_fix_topic',
        default_value='/sub/mosaic/fix',
        description='Sub GNSS NavSatFix topic (optional)'
    )
    gnss_main_gga_arg = DeclareLaunchArgument(
        'gnss_main_gga_topic',
        default_value='/main/mosaic/gga',
        description='Main GNSS GGA topic (optional)'
    )
    gnss_sub_gga_arg = DeclareLaunchArgument(
        'gnss_sub_gga_topic',
        default_value='/sub/mosaic/gga',
        description='Sub GNSS GGA topic (optional)'
    )
    use_single_gnss_imu_arg = DeclareLaunchArgument(
        'use_single_gnss_imu',
        default_value='true',
        description='Use single GNSS + IMU pose publisher instead of gnss_compass'
    )

    # Debug visualization toggle
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='true',
        description='Enable publishing debug images and markers for YabLoc'
    )

    # Delay to ensure /clock (sim time) is available before YabLoc starts
    yabloc_start_delay_arg = DeclareLaunchArgument(
        'yabloc_start_delay_sec',
        default_value='1.0',
        description='Delay seconds before starting YabLoc group (for /clock readiness)'
    )

    # Camera frame configuration (optional static TF; by default rely on URDF/robot_state_publisher)
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_optical_frame',
        description='Camera child frame (e.g., camera_optical_frame)'
    )
    camera_parent_frame_arg = DeclareLaunchArgument(
        'camera_parent_frame',
        default_value='base_link',
        description='Parent frame for camera static TF (e.g., base_link)'
    )
    add_camera_static_tf_arg = DeclareLaunchArgument(
        'add_camera_static_tf',
        default_value='true',
        description='Add extra static TF base_link->camera (default false; prefer URDF)'
    )

    # Optional camera extrinsic overrides for static TF (meters, radians)
    camera_tx_arg = DeclareLaunchArgument('camera_tx', default_value='0.0', description='camera x (m)')
    camera_ty_arg = DeclareLaunchArgument('camera_ty', default_value='0.0', description='camera y (m)')
    camera_tz_arg = DeclareLaunchArgument('camera_tz', default_value='-0.5', description='camera z (m)')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0', description='camera roll (rad)')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0', description='camera pitch (rad)')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0', description='camera yaw (rad)')

    # Image-processing tuning (graph_segment)
    graph_target_height_ratio_arg = DeclareLaunchArgument(
        'graph_target_height_ratio', default_value='0.90',
        description='graph_segment: height ratio for candidate road surface')
    graph_similarity_score_threshold_arg = DeclareLaunchArgument(
        'graph_similarity_score_threshold', default_value='0.90',
        description='graph_segment: similarity threshold for additional pickup')
    graph_k_arg = DeclareLaunchArgument(
        'graph_k', default_value='500.0',
        description='graph_segment: k parameter')
    graph_min_size_arg = DeclareLaunchArgument(
        'graph_min_size', default_value='150.0',
        description='graph_segment: min_size parameter')

    # Image-processing tuning (segment_filter)
    seg_min_segment_length_arg = DeclareLaunchArgument(
        'seg_min_segment_length', default_value='2.5',
        description='segment_filter: minimum segment length [m]')
    seg_max_lateral_distance_arg = DeclareLaunchArgument(
        'seg_max_lateral_distance', default_value='7.0',
        description='segment_filter: maximum lateral distance [m]')
    seg_max_segment_distance_arg = DeclareLaunchArgument(
        'seg_max_segment_distance', default_value='30.0',
        description='segment_filter: maximum segment distance [m]')

    # Optional calibration parameter file for undistort
    calib_param_path = PathJoinSubstitution([
        FindPackageShare('yabloc_image_processing'), 'config', 'camera_intrinsics.template.yaml'
    ])
    calib_param_arg = DeclareLaunchArgument(
        'calib_param_path',
        default_value=calib_param_path,
        description='Camera calibration parameter file for undistort node'
    )

    # Define the path to the map file using substitutions
    map_path_substitution = PathJoinSubstitution([
        FindPackageShare('localization_core'), 'data', LaunchConfiguration('map_name'),
        LaunchConfiguration('map_osm_file')
    ])

    rviz_config_path = PathJoinSubstitution([FindPackageShare('autocar_utils'), 'rviz', 'map.rviz'])

    # Define paths for sample_map test
    xacro_file = os.path.join(erp42_visualizer_share_dir, 'urdf', 'erp42.urdf.xacro')

    # XACRO processing
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = doc.toxml()
    robot_description_content = ParameterValue(robot_description_xml, value_type=str)

    robot_description_params = {'robot_description': robot_description_content, 'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        # --- Arguments ---
        map_name_arg,
        map_osm_file_arg,
        map_origin_lat_arg,
        map_origin_lon_arg,
        use_sim_time_arg,
        camera_image_topic_arg,
        camera_info_topic_arg,
        gnss_pose_topic_arg,
        gnss_main_fix_arg,
        gnss_sub_fix_arg,
        gnss_main_gga_arg,
        gnss_sub_gga_arg,
        use_single_gnss_imu_arg,
        fix_velocity_topic_arg,
        yaw_rate_sign_arg,
        enable_debug_arg,
        yabloc_start_delay_arg,
        camera_frame_arg,
        camera_parent_frame_arg,
        add_camera_static_tf_arg,
        camera_tx_arg,
        camera_ty_arg,
        camera_tz_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        graph_target_height_ratio_arg,
        graph_similarity_score_threshold_arg,
        graph_k_arg,
        graph_min_size_arg,
        seg_min_segment_length_arg,
        seg_max_lateral_distance_arg,
        seg_max_segment_distance_arg,
        calib_param_arg,
        DeclareLaunchArgument(
            'lanelet2_map_path',
            default_value=map_path_substitution,
            description='Path to the lanelet2 map file'
        ),

        # --- Nodes ---
        # 1. World->Map TF via autocar_tf_publisher (align map with UTM origin)
        Node(
            package='localization_core', executable='autocar_tf_publisher', name='autocar_tf_publisher',
            parameters=[{
                'map_origin_lat': LaunchConfiguration('map_origin_lat'),
                'map_origin_lon': LaunchConfiguration('map_origin_lon'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Optional base_link -> camera static TF (disabled by default; rely on URDF)
        GroupAction(actions=[
            Node(
                package='tf2_ros', executable='static_transform_publisher', name='static_base_camera',
                arguments=[
                    LaunchConfiguration('camera_tx'),
                    LaunchConfiguration('camera_ty'),
                    LaunchConfiguration('camera_tz'),
                    LaunchConfiguration('camera_roll'),
                    LaunchConfiguration('camera_pitch'),
                    LaunchConfiguration('camera_yaw'),
                    LaunchConfiguration('camera_parent_frame'),
                    LaunchConfiguration('camera_frame')
                ]
            ),
        ], condition=IfCondition(LaunchConfiguration('add_camera_static_tf'))),

        # 2. Map Loader Node
        Node(
            package='autoware_map_loader',
            executable='autoware_lanelet2_map_loader',
            name='lanelet2_map_loader',
            parameters=[{
                'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                'map_projector_type': 'UTM',
                'latitude': LaunchConfiguration('map_origin_lat'),
                'longitude': LaunchConfiguration('map_origin_lon'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # 3. Map Visualizer Node (Publishing markers in 'world' frame)
        Node(
            package='autoware_lanelet2_map_visualizer',
            executable='lanelet2_map_visualization',
            name='lanelet2_map_visualizer',
            parameters=[{'map_frame': 'map', 'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('input/lanelet2_map', '/map/vector_map'),
                ('output/lanelet2_map_marker', '/map/lanelet2_map_viz'),
                ('output/lanelet2_map_marker_highlight', '/map/lanelet2_map_viz_highlight')
            ]
        ),

        # 4. Robot State Publisher for vehicle model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # 5. Ackermann to Joint States for vehicle model
        Node(
            package='erp42_visualizer',
            executable='ackermann_to_joint_states.py',
            name='ackermann_to_joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 6. Lanelet Click Planner
        Node(
            package='localization_core',
            executable='global_click_planner',
            name='lanelet_click_planner',
            parameters=[{
                'map_origin.lat': LaunchConfiguration('map_origin_lat'),
                'map_origin.lon': LaunchConfiguration('map_origin_lon'),
                'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
                'allowed_lanelet_subtypes': ['road'],
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        ),

        # 6.6 YabLoc pipeline (enabled)
        TimerAction(period=1.0, actions=[
        GroupAction(
            actions=[
                # Debug: lanelet2 overlay (image-space visualization)
                Node(
                    package='yabloc_image_processing',
                    executable='yabloc_lanelet2_overlay_node',
                    name='yabloc_lanelet2_overlay',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    remappings=[
                        ('~/input/image_raw', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'),
                        ('~/input/camera_info', '/localization/pose_estimator/yabloc/image_processing/undistorted/camera_info'),
                        ('~/input/projected_line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/projected_line_segments_cloud'),
                        ('~/input/pose', '/localization/pose_selected_pose'),
                        ('~/input/ground', '/localization/pose_estimator/yabloc/map/ground'),
                        ('~/input/ll2_road_marking', '/localization/pose_estimator/yabloc/map/ll2_road_marking'),
                        ('~/input/ll2_sign_board', '/localization/pose_estimator/yabloc/map/ll2_sign_board'),
                        ('~/debug/lanelet2_overlay_image', '/localization/yabloc/image_processing/lanelet2_overlay_image'),
                        ('~/debug/projected_marker', '/localization/yabloc/image_processing/projected_marker')
                    ]
                ),
                # Provide twist_with_covariance from u-blox fix velocity + IMU yawrate
                Node(
                    package='localization_core',
                    executable='fixvel_imu_to_twist',
                    name='fixvel_imu_to_twist',
                    parameters=[{
                        'fix_velocity_topic': LaunchConfiguration('fix_velocity_topic'),
                        'imu_topic': '/imu/data',
                        'output_topic': '/localization/twist_estimator/twist_with_covariance',
                        'sync_timeout_sec': 1.0,
                        'max_skew_sec': 5.0,
                        'allow_fix_only': True,
                        'use_speed_magnitude': True,
                        'yaw_rate_sign': LaunchConfiguration('yaw_rate_sign')
                    }]
                ),
                # Seed initial pose to predictor from GNSS once
                Node(
                    package='localization_core',
                    executable='gnss_to_initialpose',
                    name='gnss_to_initialpose',
                    parameters=[{
                        'gnss_pose_topic': LaunchConfiguration('gnss_pose_topic'),
                        'initialpose_topic': '/initialpose3d',
                        'once': True
                    }]
                ),
                # Provide PoseStamped for modules needing PoseStamped (convert selected PoseCov)
                Node(
                    package='localization_core',
                    executable='pose_cov_to_pose',
                    name='pose_cov_to_pose',
                    parameters=[{
                        'input_topic': '/localization/pose_selected',
                        'output_topic': '/localization/pose_selected_pose'
                    }]
                ),

                # yabloc_common: ground height server (avoid PF pose circular dependency)
                Node(
                    package='yabloc_common',
                    executable='yabloc_ground_server_node',
                    name='yabloc_ground_server',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_common'), 'config', 'ground_server.param.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/vector_map', '/map/vector_map'),
                        ('~/input/pose', '/localization/pose_selected_pose'),
                        ('~/output/height', '/localization/pose_estimator/yabloc/map/height'),
                        ('~/output/ground', '/localization/pose_estimator/yabloc/map/ground'),
                        ('~/debug/ground_markers', '/localization/pose_estimator/yabloc/map/ground_markers'),
                        ('~/debug/ground_status', '/localization/pose_estimator/yabloc/map/ground_status'),
                        ('~/debug/near_cloud', '/localization/pose_estimator/yabloc/map/near_cloud')
                    ]
                ),
                # yabloc_common: lanelet2 decomposer
                Node(
                    package='yabloc_common',
                    executable='yabloc_ll2_decomposer_node',
                    name='yabloc_ll2_decomposer',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_common'), 'config', 'll2_decomposer.param.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/vector_map', '/map/vector_map'),
                        ('~/output/ll2_road_marking', '/localization/pose_estimator/yabloc/map/ll2_road_marking'),
                        ('~/output/ll2_sign_board', '/localization/pose_estimator/yabloc/map/ll2_sign_board'),
                        ('~/output/ll2_bounding_box', '/localization/pose_estimator/yabloc/map/ll2_bounding_box'),
                        ('~/debug/sign_board_marker', '/localization/pose_estimator/yabloc/map/sign_board_marker')
                    ]
                ),

                # yabloc_image_processing: undistort
                Node(
                    package='yabloc_image_processing',
                    executable='yabloc_undistort_node',
                    name='yabloc_undistort',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_image_processing'), 'config', 'undistort.param.yaml'),
                        LaunchConfiguration('calib_param_path'),
                        {'override_frame_id': LaunchConfiguration('camera_frame')},
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/image_raw', LaunchConfiguration('camera_image_topic')),
                        ('~/input/camera_info', LaunchConfiguration('camera_info_topic')),
                        ('~/output/resized_image', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'),
                        ('~/output/resized_info', '/localization/pose_estimator/yabloc/image_processing/undistorted/camera_info')
                    ]
                ),
                # line segment detector
                Node(
                    package='yabloc_image_processing',
                    executable='yabloc_line_segment_detector_node',
                    name='yabloc_line_segment_detector',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    remappings=[
                        ('~/input/image_raw', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'),
                        ('~/debug/image_with_line_segments', '/localization/yabloc/image_processing/image_with_line_segments'),
                        ('~/output/line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/line_segments_cloud')
                    ]
                ),
                # graph-based segmentation
                Node(
                    package='yabloc_image_processing',
                    executable='yabloc_graph_segment_node',
                    name='yabloc_graph_segment',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_image_processing'), 'config', 'graph_segment.param.yaml'),
                        {
                            'target_height_ratio': LaunchConfiguration('graph_target_height_ratio'),
                            'similarity_score_threshold': LaunchConfiguration('graph_similarity_score_threshold'),
                            'k': LaunchConfiguration('graph_k'),
                            'min_size': LaunchConfiguration('graph_min_size')
                        },
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/image_raw', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'),
                        ('~/output/mask_image', '/localization/pose_estimator/yabloc/image_processing/graph_segmented'),
                        ('~/debug/segmented_image', '/localization/yabloc/image_processing/segmented_image')
                    ]
                ),
                # segment filter and projection
                Node(
                    package='yabloc_image_processing',
                    executable='yabloc_segment_filter_node',
                    name='yabloc_segment_filter',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_image_processing'), 'config', 'segment_filter.param.yaml'),
                        {
                            'min_segment_length': LaunchConfiguration('seg_min_segment_length'),
                            'max_segment_distance': LaunchConfiguration('seg_max_segment_distance'),
                            'max_lateral_distance': LaunchConfiguration('seg_max_lateral_distance'),
                            'image_size': 800
                        },
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/line_segments_cloud'),
                        ('~/input/graph_segmented', '/localization/pose_estimator/yabloc/image_processing/graph_segmented'),
                        ('~/input/camera_info', '/localization/pose_estimator/yabloc/image_processing/undistorted/camera_info'),
                        ('~/output/projected_line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/projected_line_segments_cloud'),
                        ('~/debug/projected_image', '/localization/pose_estimator/yabloc/image_processing/projected_image'),
                        # disable debug line segments cloud publication on the main tree
                        ('~/debug/debug_line_segments', '/localization/pose_estimator/yabloc/image_processing/debug/disabled_line_segments_cloud')
                    ]
                ),

                # particle filter - predictor
                Node(
                    package='yabloc_particle_filter',
                    executable='yabloc_predictor_node',
                    name='yabloc_predictor',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_particle_filter'), 'config', 'predictor.param.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/initialpose', '/initialpose3d'),
                        ('~/input/twist_with_covariance', '/localization/twist_estimator/twist_with_covariance'),
                        ('~/input/height', '/localization/pose_estimator/yabloc/map/height'),
                        ('~/input/weighted_particles', 'weighted_particles'),
                        ('~/output/pose_with_covariance', '/localization/pose_estimator/pose_with_covariance'),
                        ('~/output/pose', '/localization/pose_estimator/yabloc/pf/pose'),
                        ('~/output/predicted_particles', 'predicted_particles'),
                        ('~/debug/init_marker', 'init_marker'),
                        ('~/debug/particles_marker_array', '/localization/yabloc/pf/predicted_particle_marker'),
                        ('~/input/ekf_pose', '/sensing/gnss/pose_with_covariance'),
                        ('~/yabloc_trigger_srv', '/localization/pose_estimator/yabloc/pf/yabloc_trigger_srv')
                    ]
                ),
                # particle filter - camera corrector
                Node(
                    package='yabloc_particle_filter',
                    executable='yabloc_camera_particle_corrector_node',
                    name='yabloc_camera_corrector',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_particle_filter'), 'config', 'camera_particle_corrector.param.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'enabled_at_first': True}
                    ],
                    remappings=[
                        ('~/input/predicted_particles', 'predicted_particles'),
                        ('~/input/line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/projected_line_segments_cloud'),
                        ('~/input/ll2_road_marking', '/localization/pose_estimator/yabloc/map/ll2_road_marking'),
                        ('~/input/ll2_bounding_box', '/localization/pose_estimator/yabloc/map/ll2_bounding_box'),
                        ('~/input/pose', '/localization/pose_selected_pose'),
                        ('~/output/weighted_particles', 'weighted_particles'),
                        ('~/debug/cost_map_range', '/localization/yabloc/pf/gnss/range_marker'),
                        ('~/debug/cost_map_image', '/localization/yabloc/pf/cost_map_image'),
                        ('~/debug/match_image', '/localization/yabloc/pf/match_image'),
                        ('~/debug/scored_cloud', '/localization/yabloc/pf/scored_cloud'),
                        ('~/debug/scored_post_cloud', '/localization/yabloc/pf/scored_post_cloud'),
                        ('~/debug/state_string', '/localization/yabloc/pf/state_string'),
                        ('~/debug/particles_marker_array', '/localization/yabloc/pf/predicted_particle_marker'),
                        ('~/switch_srv', '/localization/yabloc/pf/camera_corrector_switch')
                    ]
                ),
                # particle filter - GNSS corrector
                Node(
                    package='yabloc_particle_filter',
                    executable='yabloc_gnss_particle_corrector_node',
                    name='yabloc_gnss_corrector',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_particle_filter'), 'config', 'gnss_particle_corrector.param.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/predicted_particles', 'predicted_particles'),
                        ('~/input/height', '/localization/pose_estimator/yabloc/map/height'),
                        ('~/input/pose_with_covariance', LaunchConfiguration('gnss_pose_topic')),
                        ('~/output/weighted_particles', 'weighted_particles'),
                        ('~/debug/gnss_range_marker', '/localization/yabloc/pf/gnss_range_marker'),
                        ('~/debug/particles_marker_array', '/localization/yabloc/pf/gnss_particles_marker')
                    ]
                ),
                # yabloc monitor (optional)
                Node(
                    package='yabloc_monitor',
                    executable='yabloc_monitor_node',
                    name='yabloc_monitor',
                    parameters=[
                        os.path.join(get_package_share_directory('yabloc_monitor'), 'config', 'yabloc_monitor.param.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('~/input/yabloc_pose', '/localization/pose_estimator/yabloc/pf/pose')
                    ]
                ),

                # GNSS Compass (dual GNSS) or Single GNSS+IMU
                Node(
                    package='gnss_compass',
                    executable='gnss_compass',
                    name='gnss_compass',
                    parameters=[
                        os.path.join(get_package_share_directory('gnss_compass'), 'config', 'gnss_compass.yaml')
                    ],
                    remappings=[
                        ('main_gnss_fix', LaunchConfiguration('gnss_main_fix_topic')),
                        ('sub_gnss_fix', LaunchConfiguration('gnss_sub_fix_topic')),
                        ('main_gnss_gga', LaunchConfiguration('gnss_main_gga_topic')),
                        ('sub_gnss_gga', LaunchConfiguration('gnss_sub_gga_topic')),
                        ('gnss_compass_pose', 'gnss_compass_pose'),
                        ('gnss_compass_pose_with_covariance', LaunchConfiguration('gnss_pose_topic'))
                    ],
                    condition=UnlessCondition(LaunchConfiguration('use_single_gnss_imu'))
                ),
                Node(
                    package='localization_core',
                    executable='single_gnss_imu_pose',
                    name='single_gnss_imu_pose',
                    parameters=[{
                        'input_fix_topic': LaunchConfiguration('gnss_main_fix_topic'),
                        'input_imu_topic': '/imu/data',
                        'map_origin_lat': LaunchConfiguration('map_origin_lat'),
                        'map_origin_lon': LaunchConfiguration('map_origin_lon'),
                        'output_pose_topic': LaunchConfiguration('gnss_pose_topic'),
                        'use_sim_time': LaunchConfiguration('use_sim_time')
                    }],
                    condition=IfCondition(LaunchConfiguration('use_single_gnss_imu'))
                ),

        # Publish TF from selected pose to base_link (can be disabled)
                Node(
                    package='localization_core',
                    executable='pose_mux_to_tf',
                    name='pose_mux_to_tf',
                    parameters=[{
                        'yabloc_pose_topic': '/localization/pose_estimator/pose_with_covariance',
                        'gnss_pose_topic': LaunchConfiguration('gnss_pose_topic'),
                        'selected_pose_topic': '/localization/pose_selected',
                        'parent_frame': 'map',
                        'child_frame': 'base_link',
                'yabloc_timeout_sec': 0.5,
                'publish_tf': True
                    }]
                )
            ]
        )
        ]),

        # NOTE: /autocar/location은 odometry_ekf가 단독 발행하도록 유지 (YabLoc에서 덮어쓰지 않음)

        # 7. RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])


