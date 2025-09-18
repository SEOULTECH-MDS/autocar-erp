from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 실제 대회용 오도메트리 노드
        Node(
            package='localization_core',
            executable='odometry',
            name='odometry_node',
            output='screen',
        ),
        
        # 주차 패키지 노드들
        Node(
            package='parking_planner',
            executable='parking_rubber_cones_adapter',
            name='parking_rubber_cones_adapter',
            output='screen',
        ),
        Node(
            package='parking_planner',
            executable='parking_cones_mapper',
            name='parking_cones_mapper',
            output='screen',
        ),
        Node(
            package='parking_planner',
            executable='parking_area_detector',
            name='parking_area_detector',
            output='screen',
        ),
        
        # 플래너 노드
        Node(
            package='planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[
                # 기본 주차 파라미터
                {'yaw_offset_deg': 15.0},
                {'s_overshoot': 0.5},
                {'clear_lateral': 0.3},
                {'front_margin': 0.5},
                
                # 스테이지 2 파라미터
                {'stage2_guided': True},
                {'stage2_k_yaw': 0.0},
                {'stage2_k_lat': 1.0},
                {'stage2_inside_margin': 0.2},
                {'stage2_max_steps': 100},
                {'stage2_lookahead': 1.0},
                
                # 스테이지 3 파라미터
                {'stage3_yaw_tol_deg': 5.0},
                {'stage3_turn_radius': 2.0},
                {'stage3_forward': 0.4},
                
                # 차량 파라미터
                {'vehicle_width': 1.8},
                {'vehicle_length': 2.02},
                {'turn_radius': 3.0},
                {'safety_margin': 0.3},
                
                # 경로 생성 파라미터
                {'path_resolution': 0.1},
                {'auto_advance': True},
                {'test_mode_immediate_s_curve': False},  # 테스트 모드: False=정상, True=S자 경로 즉시 생성

                {'show_stage1_path': True},
                {'publish_unified_waypoints': False},  # 현재 스테이지만 퍼블리시
                
                # S-Curve 파라미터
                {'s_curve_enabled': True},
                {'s_curve_radius1': 2.0},
                {'s_curve_radius2': 2.0},
                {'s_curve_middle_offset': 0.5},
                {'s_curve_smoothing': True},
                {'s_curve_resolution': 0.1},
                {'s_curve_collision_resolution': 0.05},
                {'s_curve_vehicle_radius': 1.0},
                
                # 프레임 설정
                {'frame_id': 'world'},
            ],
        ),
    ])
