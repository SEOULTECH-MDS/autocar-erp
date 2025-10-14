from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    erp42_visualizer_share_dir = get_package_share_directory('erp42_visualizer')

    map_name_arg = DeclareLaunchArgument('map_name', default_value='kcity_v6', description='map name')
    map_osm_file_arg = DeclareLaunchArgument('map_osm_file', default_value='lanelet2_map.osm')
    map_origin_lat_arg = DeclareLaunchArgument('map_origin_lat', default_value='37.239205')
    map_origin_lon_arg = DeclareLaunchArgument('map_origin_lon', default_value='126.773193')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    camera_image_topic_arg = DeclareLaunchArgument('camera_image_topic', default_value='/usb_cam_1/image_raw')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/usb_cam_1/camera_info')
    gnss_pose_topic_arg = DeclareLaunchArgument('gnss_pose_topic', default_value='/sensing/gnss/pose_with_covariance')
    enable_debug_arg = DeclareLaunchArgument('enable_debug', default_value='true')

    map_path_substitution = PathJoinSubstitution([
        FindPackageShare('localization_core'), 'data', LaunchConfiguration('map_name'),
        LaunchConfiguration('map_osm_file')
    ])

    rviz_config_path = PathJoinSubstitution([FindPackageShare('autocar_utils'), 'rviz', 'map.rviz'])

    xacro_file = os.path.join(erp42_visualizer_share_dir, 'urdf', 'erp42.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = doc.toxml()
    robot_description_content = ParameterValue(robot_description_xml, value_type=str)
    robot_description_params = {'robot_description': robot_description_content, 'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        map_name_arg, map_osm_file_arg, map_origin_lat_arg, map_origin_lon_arg,
        use_sim_time_arg, camera_image_topic_arg, camera_info_topic_arg, gnss_pose_topic_arg, enable_debug_arg,
        DeclareLaunchArgument('lanelet2_map_path', default_value=map_path_substitution),

        Node(
            package='localization_core', executable='autocar_tf_publisher', name='autocar_tf_publisher',
            parameters=[{'map_origin_lat': LaunchConfiguration('map_origin_lat'), 'map_origin_lon': LaunchConfiguration('map_origin_lon'), 'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='autoware_map_loader', executable='autoware_lanelet2_map_loader', name='lanelet2_map_loader',
            parameters=[{'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'), 'map_projector_type': 'UTM', 'latitude': LaunchConfiguration('map_origin_lat'), 'longitude': LaunchConfiguration('map_origin_lon'), 'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='autoware_lanelet2_map_visualizer', executable='lanelet2_map_visualization', name='lanelet2_map_visualizer',
            parameters=[{'map_frame': 'world', 'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('input/lanelet2_map', '/map/vector_map'), ('output/lanelet2_map_marker', '/map/lanelet2_map_viz'), ('output/lanelet2_map_marker_highlight', '/map/lanelet2_map_viz_highlight')]
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[robot_description_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='erp42_visualizer', executable='ackermann_to_joint_states.py', name='ackermann_to_joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='localization_core', executable='global_click_planner', name='lanelet_click_planner',
            parameters=[{'map_origin.lat': LaunchConfiguration('map_origin_lat'), 'map_origin.lon': LaunchConfiguration('map_origin_lon'), 'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'), 'allowed_lanelet_subtypes': ['road'], 'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        GroupAction(actions=[
            Node(
                package='yabloc_common', executable='yabloc_ground_server_node', name='yabloc_ground_server',
                parameters=[os.path.join(get_package_share_directory('yabloc_common'), 'config', 'ground_server.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/vector_map', '/map/vector_map'), ('~/input/pose', '/localization/pose_estimator/yabloc/pf/pose'), ('~/output/height', '/localization/pose_estimator/yabloc/map/height'), ('~/output/ground', '/localization/pose_estimator/yabloc/map/ground'), ('~/debug/ground_markers', '/localization/pose_estimator/yabloc/map/ground_markers'), ('~/debug/ground_status', '/localization/pose_estimator/yabloc/map/ground_status'), ('~/debug/near_cloud', '/localization/pose_estimator/yabloc/map/near_cloud')]
            ),
            Node(
                package='yabloc_common', executable='yabloc_ll2_decomposer_node', name='yabloc_ll2_decomposer',
                parameters=[os.path.join(get_package_share_directory('yabloc_common'), 'config', 'll2_decomposer.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/vector_map', '/map/vector_map'), ('~/output/ll2_road_marking', '/localization/pose_estimator/yabloc/map/ll2_road_marking'), ('~/output/ll2_sign_board', '/localization/pose_estimator/yabloc/map/ll2_sign_board'), ('~/output/ll2_bounding_box', '/localization/pose_estimator/yabloc/map/ll2_bounding_box'), ('~/debug/sign_board_marker', '/localization/pose_estimator/yabloc/map/sign_board_marker')]
            ),
            Node(
                package='yabloc_image_processing', executable='yabloc_undistort_node', name='yabloc_undistort',
                parameters=[os.path.join(get_package_share_directory('yabloc_image_processing'), 'config', 'undistort.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/image_raw', LaunchConfiguration('camera_image_topic')), ('~/input/camera_info', LaunchConfiguration('camera_info_topic')), ('~/output/resized_image', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'), ('~/output/resized_info', '/localization/pose_estimator/yabloc/image_processing/undistorted/camera_info')]
            ),
            Node(
                package='yabloc_image_processing', executable='yabloc_line_segment_detector_node', name='yabloc_line_segment_detector',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/image_raw', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'), ('~/debug/image_with_line_segments', '/localization/pose_estimator/yabloc/image_processing/image_with_line_segments'), ('~/output/line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/line_segments_cloud')]
            ),
            Node(
                package='yabloc_image_processing', executable='yabloc_graph_segment_node', name='yabloc_graph_segment',
                parameters=[os.path.join(get_package_share_directory('yabloc_image_processing'), 'config', 'graph_segment.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/image_raw', '/localization/pose_estimator/yabloc/image_processing/undistorted/image_raw'), ('~/output/mask_image', '/localization/pose_estimator/yabloc/image_processing/graph_segmented'), ('~/debug/segmented_image', '/localization/pose_estimator/yabloc/image_processing/segmented_image')]
            ),
            Node(
                package='yabloc_image_processing', executable='yabloc_segment_filter_node', name='yabloc_segment_filter',
                parameters=[os.path.join(get_package_share_directory('yabloc_image_processing'), 'config', 'segment_filter.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/line_segments_cloud'), ('~/input/graph_segmented', '/localization/pose_estimator/yabloc/image_processing/graph_segmented'), ('~/input/camera_info', '/localization/pose_estimator/yabloc/image_processing/undistorted/camera_info'), ('~/output/projected_line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/projected_line_segments_cloud'), ('~/debug/projected_image', '/localization/pose_estimator/yabloc/image_processing/projected_image'), ('~/debug/debug_line_segments', '/localization/pose_estimator/yabloc/image_processing/debug/line_segments_cloud')]
            ),
            Node(
                package='yabloc_particle_filter', executable='yabloc_predictor_node', name='yabloc_predictor',
                parameters=[os.path.join(get_package_share_directory('yabloc_particle_filter'), 'config', 'predictor.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/initialpose', '/initialpose3d'), ('~/input/twist_with_covariance', '/localization/twist_estimator/twist_with_covariance'), ('~/input/height', '/localization/pose_estimator/yabloc/map/height'), ('~/input/weighted_particles', 'weighted_particles'), ('~/output/pose_with_covariance', '/localization/pose_estimator/pose_with_covariance'), ('~/output/pose', '/localization/pose_estimator/yabloc/pf/pose'), ('~/output/predicted_particles', 'predicted_particles'), ('~/debug/init_marker', 'init_marker'), ('~/debug/particles_marker_array', '/localization/yabloc/pf/predicted_particle_marker'), ('~/input/ekf_pose', '/localization/pose_estimator/pose_with_covariance'), ('~/yabloc_trigger_srv', '/localization/pose_estimator/yabloc/pf/yabloc_trigger_srv')]
            ),
            Node(
                package='yabloc_particle_filter', executable='yabloc_camera_particle_corrector_node', name='yabloc_camera_corrector',
                parameters=[os.path.join(get_package_share_directory('yabloc_particle_filter'), 'config', 'camera_particle_corrector.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/predicted_particles', 'predicted_particles'), ('~/input/line_segments_cloud', '/localization/pose_estimator/yabloc/image_processing/projected_line_segments_cloud'), ('~/input/ll2_road_marking', '/localization/pose_estimator/yabloc/map/ll2_road_marking'), ('~/input/ll2_bounding_box', '/localization/pose_estimator/yabloc/map/ll2_bounding_box'), ('~/input/pose', '/localization/pose_estimator/yabloc/pf/pose'), ('~/output/weighted_particles', 'weighted_particles'), ('~/debug/cost_map_range', '/localization/yabloc/pf/cost_map_range'), ('~/debug/cost_map_image', '/localization/yabloc/pf/cost_map_image'), ('~/debug/match_image', '/localization/yabloc/pf/match_image'), ('~/debug/scored_cloud', '/localization/yabloc/pf/scored_cloud'), ('~/debug/scored_post_cloud', '/localization/yabloc/pf/scored_post_cloud'), ('~/debug/state_string', '/localization/yabloc/pf/state_string'), ('~/debug/particles_marker_array', '/localization/yabloc/pf/camera_particles_marker'), ('~/switch_srv', '/localization/yabloc/pf/camera_corrector_switch')]
            ),
            Node(
                package='yabloc_particle_filter', executable='yabloc_gnss_particle_corrector_node', name='yabloc_gnss_corrector',
                parameters=[os.path.join(get_package_share_directory('yabloc_particle_filter'), 'config', 'gnss_particle_corrector.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/predicted_particles', 'predicted_particles'), ('~/input/height', '/localization/pose_estimator/yabloc/map/height'), ('~/input/pose_with_covariance', LaunchConfiguration('gnss_pose_topic')), ('~/output/weighted_particles', 'weighted_particles'), ('~/debug/gnss_range_marker', '/localization/yabloc/pf/gnss_range_marker'), ('~/debug/particles_marker_array', '/localization/yabloc/pf/gnss_particles_marker')]
            ),
            Node(
                package='yabloc_monitor', executable='yabloc_monitor_node', name='yabloc_monitor',
                parameters=[os.path.join(get_package_share_directory('yabloc_monitor'), 'config', 'yabloc_monitor.param.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('~/input/yabloc_pose', '/localization/pose_estimator/yabloc/pf/pose')]
            ),

        ]),
        # Publish TF from YabLoc pose to base_link
        Node(
            package='localization_core', executable='pose_to_tf', name='yabloc_pose_to_tf',
            parameters=[{'pose_topic': '/localization/pose_estimator/pose_with_covariance', 'parent_frame': 'map', 'child_frame': 'base_link'}]
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', rviz_config_path], output='screen'
        )
    ])


