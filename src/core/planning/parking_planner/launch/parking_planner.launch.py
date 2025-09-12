#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    input_markers = LaunchConfiguration('input_markers', default='/sensor_fusion/tracked_rubber_cones')
    output_obstacles = LaunchConfiguration('output_obstacles', default='/parking/cones_obstacles')
    output_markers = LaunchConfiguration('output_markers', default='/parking/cones_obstacles_markers')
    target_frame = LaunchConfiguration('target_frame', default='map')
    cone_radius_m = LaunchConfiguration('cone_radius_m', default='0.15')
    marker_alpha = LaunchConfiguration('marker_alpha', default='0.5')
    right_is_negative_y = LaunchConfiguration('right_is_negative_y', default='true')
    debug_enabled = LaunchConfiguration('debug_enabled', default='true')

    # detector debug arg (separate from adapter's debug)
    detector_debug_enabled = LaunchConfiguration('detector_debug_enabled', default='true')

    # ROI params (shared by mapper and detector)
    roi_y_min_m = LaunchConfiguration('roi_y_min_m', default='-3.0')
    roi_y_max_m = LaunchConfiguration('roi_y_max_m', default='0.0')

    # Coerce boolean parameters to actual bools
    use_sim_time_bool = ParameterValue(use_sim_time, value_type=bool)
    right_is_negative_y_bool = ParameterValue(right_is_negative_y, value_type=bool)
    debug_enabled_bool = ParameterValue(debug_enabled, value_type=bool)
    detector_debug_enabled_bool = ParameterValue(detector_debug_enabled, value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('input_markers', default_value='/sensor_fusion/tracked_rubber_cones'),
        DeclareLaunchArgument('output_obstacles', default_value='/parking/cones_obstacles'),
        DeclareLaunchArgument('output_markers', default_value='/parking/cones_obstacles_markers'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('cone_radius_m', default_value='0.15'),
        DeclareLaunchArgument('marker_alpha', default_value='0.5'),
        DeclareLaunchArgument('right_is_negative_y', default_value='true'),
        DeclareLaunchArgument('debug_enabled', default_value='true'),
        DeclareLaunchArgument('detector_debug_enabled', default_value='true'),
        DeclareLaunchArgument('roi_y_min_m', default_value='-3.0'),
        DeclareLaunchArgument('roi_y_max_m', default_value='0.0'),

        Node(
            package='parking_planner',
            executable='parking_rubber_cones_adapter',
            name='parking_rubber_cones_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'input_markers': input_markers,
                'output_obstacles': output_obstacles,
                'output_markers': output_markers,
                'target_frame': target_frame,
                'cone_radius_m': cone_radius_m,
                'marker_alpha': marker_alpha,
                'right_is_negative_y': right_is_negative_y_bool,
                'debug_enabled': False,
            }],
        ),

        # 2) Mapper: stabilize cones and republish
        Node(
            package='parking_planner',
            executable='parking_cones_mapper',
            name='parking_cones_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'input_topic': output_obstacles,
                'output_topic': '/parking/cones_mapped',
                'markers_topic': '/parking/cones_mapped_markers',
                # merge_distance_m: 동일 콘으로 병합할 최대 거리(m)
                'merge_distance_m': 0.6,
                # ema_alpha: 지수이동평균의 알파(0~1, 높을수록 반응 빠름/노이즈 큼)
                'ema_alpha': 0.3,
                # forget_time_sec: 관측 끊긴 트랙을 삭제하기까지 대기 시간(s)
                'forget_time_sec': 2.0,
                # min_count_to_publish: 이 횟수 이상 관측된 트랙만 퍼블리시
                'min_count_to_publish': 2,
                # publish_rate_hz: 안정화 결과를 퍼블리시하는 주기(Hz)
                'publish_rate_hz': 10.0,
                # marker_alpha: 시각화 마커 투명도(0~1)
                'marker_alpha': 1.0,
                # shared ROI params
                'roi_y_min_m': roi_y_min_m,
                'roi_y_max_m': roi_y_max_m,
            }],
        ),

        # 3) Area detector
        Node(
            package='parking_planner',
            executable='parking_area_detector',
            name='parking_area_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'input_topic': '/parking/cones_mapped_in_roi',
                'debug_enabled': detector_debug_enabled_bool,
                # shared ROI params (for visualization and consistency)
                'roi_y_min_m': roi_y_min_m,
                'roi_y_max_m': roi_y_max_m,
            }],
        ),
   
    ])


