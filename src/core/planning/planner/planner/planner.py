#!/usr/bin/env python3
"""
자율주행 자동차 3단계 주차 계획 시스템

이 모듈은 자율주행 자동차가 주차 공간을 찾아 안전하게 주차하는 경로를 계획합니다.
3단계 주차 시스템을 통해 전진 → 후진 → 정렬의 순서로 주차를 수행합니다.

주요 기능:
- 3단계 주차 경로 계획 (Stage-1, Stage-2, Stage-3)
- S자 경로 지원 (부드러운 곡선 후진)
- 콘 기반 충돌 회피
- 배달 미션 지원
- 실시간 경로 발행 및 시각화

작성자: 자율주행 팀
버전: 1.0
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ObstacleArray, Obstacle, ModeState
from std_msgs.msg import Int32, Bool
import tf2_ros
import tf2_geometry_msgs


def wrap_to_pi(angle: float) -> float:
    """각도를 [-π, π] 범위로 정규화
    Args:
        angle: 정규화할 각도 [라디안]
    Returns:
        [-π, π] 범위의 각도
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    """쿼터니언에서 yaw 각도 추출 (Z-W만 사용)
    Args:
        q: geometry_msgs/Quaternion
    Returns:
        yaw 각도 [라디안]
    """
    return math.atan2(2.0 * q.z * q.w, 1.0 - 2.0 * q.z * q.z)


def yaw_to_quaternion(yaw: float):
    """yaw 각도를 쿼터니언으로 변환
    Args:
        yaw: yaw 각도 [라디안]
    Returns:
        geometry_msgs/Quaternion
    """
    from geometry_msgs.msg import Quaternion
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


# ==================== 주차 계획 시스템 ====================
# 이 파일은 자율주행 자동차의 3단계 주차 시스템을 구현합니다.
# - Stage-1: 주차 공간 앞으로 전진 이동
# - Stage-2: 주차 공간으로 후진 (S자 경로 또는 직선)
# - Stage-3: 최종 정렬 및 완료
# 
# 주요 특징:
# - 콘 기반 충돌 회피 (가상 벽 대신)
# - S자 경로 지원 (부드러운 곡선)
# - 배달 미션 지원
# - 실시간 경로 계획 및 발행


class PlannerNode(Node):
    """3단계 주차 계획 시스템 (Stage-1, Stage-2, Stage-3)

    이 노드는 자율주행 자동차가 주차 공간을 찾아 안전하게 주차하는 경로를 계획합니다.
    
    구독 토픽:
        /parking/parking_pose     (PoseStamped)    - 주차 공간 위치 정보
        /parking/open_area_idx    (Int32)          - 주차 공간 인덱스
        /parking/cones_mapped_in_roi (ObstacleArray) - 안정화된 콘 정보
        /autocar/location         (Odometry)       - 차량 현재 위치 [필수]
        /mode_state               (ModeState)      - 주차/배달 모드 선택
        /stage_selector           (Int32)          - 수동 스테이지 선택 (1,2,3)
    
    발행 토픽:
        /waypoints                (Path)           [메인] 현재 활성 스테이지의 경로
        /waypoints_points         (PoseArray)      [메인] 현재 활성 스테이지의 웨이포인트
        /reverse_flag             (Bool)           [메인] 후진 모드 플래그
        /stage1_path              (Path)           [디버그] Stage-1 경로 시각화
        /stage2_path              (Path)           [디버그] Stage-2 경로 시각화  
        /stage3_path              (Path)           [디버그] Stage-3 경로 시각화
        /stage1_goal              (PoseStamped)    [디버그] Stage-1 목표 위치
        /stage2_goal              (PoseStamped)    [디버그] Stage-2 목표 위치
        /stage3_goal              (PoseStamped)    [디버그] Stage-3 목표 위치
        /planner_debug_markers    (MarkerArray)    [디버그] 시각화 마커
    """

    def __init__(self) -> None:
        super().__init__('planner_node')

        # ==================== 주차 계획 파라미터 ====================
        
        # ==================== 공통 파라미터 ====================
        # 차량 물리적 특성
        self.declare_parameter('vehicle_width', 1.16)           # 차량 폭 [m]
        self.declare_parameter('vehicle_length', 2.02)          # 차량 길이 [m]
        self.declare_parameter('safety_margin', 0.1)            # 충돌 회피를 위한 안전 여유 [m]
        self.declare_parameter('path_resolution', 0.01)         # 경로 점들 사이의 간격 [m]
        
        # 주차 공간 기본 정보
        self.declare_parameter('default_slot_len', 5.0)         # 주차 공간 길이 기본값 [m]
        self.declare_parameter('default_slot_width', 2.5)       # 주차 공간 폭 기본값 [m]
        
        # ==================== Stage-1 (전진 이동) 파라미터 ====================
        self.declare_parameter('front_margin', 3)               # 주차 공간 전방 경계에서 추가 여유 [m]
        self.declare_parameter('clear_lateral', 1.3)            # 주차 공간 옆쪽(도로쪽) 여유 [m]
        self.declare_parameter('yaw_offset_deg', 0.0)           # Stage-1에서 좌측으로 선회할 각도 [도]
        self.declare_parameter('show_stage1_path', True)        # Stage-1 경로 시각화 여부
        
        # ==================== Stage-2 목표 계산 파라미터 ====================
        self.declare_parameter('stage2_use_map_y_offset', False)  # Stage-2 목표를 map 프레임 y 오프셋으로 설정
        self.declare_parameter('stage2_goal_offset_y', 0.0)     # map 프레임 y축 오프셋 [m]
        self.declare_parameter('stage2_back_offset', 0.0)       # 슬롯 진행축 기준 뒤로 이동 거리 [m] (대체 옵션)
        self.declare_parameter('stage2_inside_margin', 0.2)     # 주차 공간 내부 안전 여유 [m]

        # ==================== Stage-3 (최종 정렬) 파라미터 ====================
        self.declare_parameter('stage3_enabled', True)          # Stage-3 (최종 정렬) 활성화 여부
        # [사용안함] Stage-3 관련 파라미터들
        self.declare_parameter('stage3_preview', True)          # [사용안함] Stage-3 미리보기
        
        # ==================== Stage-2 경로 생성 우선순위별 파라미터 ====================
        
        # === 1순위: 작년 로직 (Old-logic) - 3개 직선 구간 경로 ===
        self.declare_parameter('oldlogic_use', True)            # 작년 로직 사용 여부 (최우선)
        self.declare_parameter('oldlogic_pre_reverse', 1.5)     # Stage-1 골에서 반대방향 직진 거리 [m]
        self.declare_parameter('oldlogic_pre_straight', 0.2)    # 초기 직선 step 크기 [m]
        self.declare_parameter('oldlogic_center_offset', 1.5)   # 구역 중심 오프셋 (주차위치에서 앞으로) [m]
        # [사용안함] 작년 로직 관련 파라미터들
        self.declare_parameter('oldlogic_min_R', 2.55)          # [사용안함] 최소 회전 반경 [m]
        self.declare_parameter('oldlogic_spot_dx', 1.15)        # [사용안함] p3 보정: 세로 이동 [m]
        self.declare_parameter('oldlogic_spot_dy', 1.20)        # [사용안함] p3 보정: 가로 이동 [m]
        self.declare_parameter('oldlogic_arc_points', 15)       # [사용안함] 각 원호 샘플 점 수
        
        # === 2순위: 하드코딩 S-curve 경로 ===
        self.declare_parameter('stage2_use_hardcoded', True)    # 하드코딩 경로 사용 (2순위)
        self.declare_parameter('hc_wall_clear', 0.35)           # 벽과 최소 간격 [m]
        self.declare_parameter('hc_straight_len', 1.6)          # 벽 따라 곧게 내려오는 길이 [m]
        self.declare_parameter('hc_c1', 0.8)                    # 베지어 제어점 길이(시작쪽) [m]
        self.declare_parameter('hc_c2', 1.2)                    # 베지어 제어점 길이(끝쪽) [m]
        self.declare_parameter('hc_goal_back', 0.8)             # 슬롯 중심에서 뒤로(−x) 얼마나 들어갈지 [m]
        self.declare_parameter('hc_inside_margin', 0.20)        # 슬롯 안쪽으로 들어갈 y 여유 [m]
        self.declare_parameter('hc_ds', 0.04)                   # 샘플 간격 [m]
        
        # === 3순위: 벽 따라 S-curve 경로 (곡률 프로파일 기반) ===
        self.declare_parameter('stage2_shape_mode', 's_curve')  # s_curve / single_turn
        self.declare_parameter('shape_total_length', 8.0)       # [m] 전체 아크길이 (매우 길게)
        self.declare_parameter('shape_kappa_max', 0.60)         # [1/m] 꺾임 강도 (매우 강하게)
        self.declare_parameter('shape_switch_pos', 0.80)        # S전환 위치 (매우 늦게)
        self.declare_parameter('shape_gamma', 4.0)              # 펄스 폭 (매우 부드럽게)
        self.declare_parameter('shape_ds', 0.08)                # [m] 샘플 간격 (더 거칠게)
        self.declare_parameter('shape_wall_clearance', 0.15)    # [m] 벽과 최소 간격 (매우 가깝게)
        self.declare_parameter('shape_wall_hold_s', 4.0)        # [m] 시작~여기까지 벽 따라 내려오기 (매우 길게)
        self.declare_parameter('shape_wall_softness', 20.0)     # 부드러운 제약 (매우 강하게)
        self.declare_parameter('wheelbase', 1.10)               # [m] 휠베이스
        self.declare_parameter('delta_max_deg', 35.0)           # [deg] 최대 조향각
        
        # === 4순위: 기존 S-curve 경로 (베지어 곡선 기반) ===
        self.declare_parameter('s_curve_enabled', True)         # S자 경로 사용 여부 (4순위)
        self.declare_parameter('s_curve_resolution', 0.1)       # S자 경로 점들 사이의 간격 [m]
        self.declare_parameter('s_curve_collision_resolution', 0.05)  # S자 경로 충돌 검사 해상도 [m]
        self.declare_parameter('s_curve_vehicle_radius', 1.0)   # S자 경로 충돌 검사용 차량 반경 [m]
        # [사용안함] 기존 S-curve 관련 파라미터들
        self.declare_parameter('s_curve_radius1', 2.0)          # [사용안함] S자 경로 첫 번째 원호 반경 [m]
        self.declare_parameter('s_curve_radius2', 2.0)          # [사용안함] S자 경로 두 번째 원호 반경 [m]
        self.declare_parameter('s_curve_middle_offset', 0.5)    # [사용안함] S자 경로 중간점 오프셋 [m]
        self.declare_parameter('s_curve_smoothing', True)       # [사용안함] 곡률 스무딩 활성화
        

        
        # ==================== 스테이지 제어 파라미터 ====================
        self.declare_parameter('auto_advance', True)            # 오도메트리 기반 자동 스테이지 전환 여부
        self.declare_parameter('test_mode_immediate_s_curve', False)  # 테스트 모드: 주차 포즈 수신 시 즉시 S자 경로 생성
        self.declare_parameter('stage_position_tolerance', 0.25)  # 스테이지 완료 위치 허용 오차 [m]
        self.declare_parameter('stage_yaw_tolerance_deg', 8.0)    # 스테이지 완료 방향 허용 오차 [도]
        self.declare_parameter('publish_unified_waypoints', False)  # 현재 스테이지만 /waypoints 퍼블리시 여부
        
        # ==================== 디버그 파라미터 ====================
        self.declare_parameter('publish_debug_paths', True)     # 디버그 경로 퍼블리싱 여부 (시각화용)
        
        # ==================== 주차 완료 파라미터 ====================
        self.declare_parameter('parking_stop_duration', 3.0)    # 주차 완료 시 정지 시간 [초]
        self.declare_parameter('parking_complete_yaw_tolerance_deg', 5.0)  # 주차 완료 방향 임계치 [도]
        
        # ==================== 배달 미션 파라미터 ====================
        self.declare_parameter('delivery_position_tolerance', 0.3)  # 배달 목표 위치 허용 오차 [m]
        self.declare_parameter('delivery_yaw_tolerance_deg', 12.0)  # 배달 목표 방향 허용 오차 [도]
        self.declare_parameter('delivery_stop_offset', 1.0)        # 표지판 앞 정지 오프셋 [m]
        self.declare_parameter('delivery_auto_activate', False)    # 배달 토픽 수신 시 자동 모드 전환 (기본 OFF)
        self.declare_parameter('enable_delivery_planning', False)  # 배달 경로 생성 전체 활성화 (기본 OFF)
        
        # ==================== [사용안함] 레거시 파라미터들 ====================
        self.declare_parameter('s_overshoot', 1.0)              # [사용안함] 슬롯 중심을 지나칠 거리 [m]
        self.declare_parameter('angle_eps_deg', 5.0)            # [사용안함] 평행/수직 분류 각도 허용치 [도]
        self.declare_parameter('prefer_near_open_slot', False)  # [사용안함] 가까운 주차 공간 선호
        self.declare_parameter('near_radius', 8.0)              # [사용안함] 가까운 주차 공간 반경 [m]
        self.declare_parameter('prefer_centerline_lateral', False)  # [사용안함] 도로 중심선 기반 측면 배치
        self.declare_parameter('centerline_left_margin', 0.1)   # [사용안함] 도로 중심선 기준 왼쪽 여유 [m]
        self.declare_parameter('turn_radius', 0.5)              # [사용안함] Stage-2 원호 경로의 회전 반경 [m]
        self.declare_parameter('stage2_k_lat', 0.8)             # [사용안함] 측면 제어 게인
        self.declare_parameter('stage2_k_yaw', 0.8)             # [사용안함] 방향 제어 게인
        self.declare_parameter('stage2_max_steps', 800)         # [사용안함] 최대 스텝 수
        self.declare_parameter('stage2_lookahead', 1.0)         # [사용안함] 전방 예측 거리 [m]
        
        # ==================== 좌표계 설정 ====================
        self.declare_parameter('frame_id', 'map')  # 기본 좌표계

        # ==================== 상태 변수 ====================
        # 주차 관련 상태
        self._parking_pose: Optional[PoseStamped] = None      # 주차 공간 위치
        self._odom: Optional[Odometry] = None                 # 차량 현재 위치 (오도메트리)
        self._parking_area_idx: Optional[int] = None          # 주차 공간 인덱스
        self._stable_cones: List[Obstacle] = []               # 안정화된 콘 정보 (충돌 회피용)
        
        # 각 스테이지별 목표 위치 저장
        self._last_stage1_goal: Optional[PoseStamped] = None  # Stage-1 목표 위치
        self._last_stage2_goal: Optional[PoseStamped] = None  # Stage-2 목표 위치
        self._last_stage3_goal: Optional[PoseStamped] = None  # Stage-3 목표 위치
        
        # 스테이지 제어
        self._stage: int = 1                                   # 현재 스테이지 (1, 2, 3)
        self._selector_stage: Optional[int] = None             # 외부 스테이지 선택기 (수동 제어)
        
        # 미션 모드
        self._mission: str = 'parking'                         # 현재 미션: 'parking' 또는 'delivery'
        
        # 배달 미션 관련 상태
        self._delivery_spots: Optional[PoseArray] = None       # 배달 표지판 후보 위치들
        self._delivery_target_sign: Optional[int] = None       # 목표 표지판 ID (3,4,5 -> B1,B2,B3)
        self._delivery_phase: str = 'pickup'                   # 배달 단계: 'pickup' -> 'dropoff'
        self._delivery_pick_target: Optional[PoseStamped] = None  # 상차 목표 위치
        self._delivery_drop_target: Optional[PoseStamped] = None  # 하차 목표 위치
        
        # 주차 완료 관련 상태
        self._parking_complete_state: str = 'normal'           # 주차 완료 상태: 'normal', 'stopping', 'completed'
        self._parking_stop_timer: Optional[float] = None       # 주차 정지 타이머 시작 시간
        self._parking_complete_triggered: bool = False         # 주차 완료 트리거 여부
        
        # ==================== 좌표 변환 설정 ====================
        self._tf_buffer = tf2_ros.Buffer()                      # TF 변환 버퍼
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)  # TF 리스너

        # ==================== 퍼블리셔 설정 ====================
        # 메인 토픽: 컨트롤러가 구독하는 표준 토픽 (현재 활성 스테이지의 경로)
        self._waypoints_pub = self.create_publisher(Path, '/waypoints', 10)
        self._waypoints_points_pub = self.create_publisher(PoseArray, '/waypoints_points', 10)
        self._reverse_flag_pub = self.create_publisher(Bool, '/reverse_flag', 10)  # 후진 모드 플래그
        self._parking_stop_flag_pub = self.create_publisher(Bool, '/parking_stop_flag', 10)  # 주차 완료 3초 정지 플래그
        self._parking_complete_flag_pub = self.create_publisher(Bool, '/parking_complete_flag', 10)  # 주차 종료 플래그

        # 디버그 토픽: 각 스테이지별 경로/목표 및 시각화 마커
        self._goal_pub = self.create_publisher(PoseStamped, '/stage1_goal', 10)           # Stage-1 목표
        self._stage2_goal_pub = self.create_publisher(PoseStamped, '/stage2_goal', 10)    # Stage-2 목표
        self._stage3_goal_pub = self.create_publisher(PoseStamped, '/stage3_goal', 10)    # Stage-3 목표
        self._path_pub = self.create_publisher(Path, '/stage1_path', 10)                  # Stage-1 경로
        self._stage2_path_pub = self.create_publisher(Path, '/stage2_path', 10)           # Stage-2 경로
        self._stage3_path_pub = self.create_publisher(Path, '/stage3_path', 10)           # Stage-3 경로
        self._vis_pub = self.create_publisher(MarkerArray, '/planner_debug_markers', 10)  # 시각화 마커
        
        # 각 스테이지별 웨이포인트 (PoseArray 형태)
        self._stage1_points_pub = self.create_publisher(PoseArray, '/stage1_waypoints', 10)
        self._stage2_points_pub = self.create_publisher(PoseArray, '/stage2_waypoints', 10)
        self._stage3_points_pub = self.create_publisher(PoseArray, '/stage3_waypoints', 10)
        
        # 배달 미션 퍼블리셔
        self._delivery_pick_path_pub = self.create_publisher(Path, '/delivery_pick_path', 10)      # 상차 경로
        self._delivery_drop_path_pub = self.create_publisher(Path, '/delivery_drop_path', 10)      # 하차 경로
        self._delivery_pick_points_pub = self.create_publisher(PoseArray, '/delivery_pick_waypoints', 10)  # 상차 웨이포인트
        self._delivery_drop_points_pub = self.create_publisher(PoseArray, '/delivery_drop_waypoints', 10)  # 하차 웨이포인트
        
        # 초기 플래그 설정
        self._publish_reverse_flag(False)  # Stage-1은 전진
        
        # 주차 완료 관련 플래그 초기화
        stop_msg = Bool()
        stop_msg.data = False
        self._parking_stop_flag_pub.publish(stop_msg)
        
        complete_msg = Bool()
        complete_msg.data = False
        self._parking_complete_flag_pub.publish(complete_msg)

        # ==================== 구독자 설정 ====================
        # 주차 관련 정보 구독
        self.create_subscription(Int32, '/parking/open_area_idx', self._on_parking_area_idx, 10)          # 주차 공간 인덱스
        self.create_subscription(PoseStamped, '/parking/parking_pose', self._on_parking_pose, 10)         # 주차 공간 위치
        self.create_subscription(ObstacleArray, '/parking/cones_mapped_in_roi', self._on_stable_cones, 10)  # 안정화된 콘 정보
        
        # 차량 위치 및 제어 관련 구독
        self.create_subscription(Odometry, '/autocar/location', self._on_odom, 20)                        # 차량 현재 위치 (오도메트리)
        self.create_subscription(Int32, '/stage_selector', self._on_stage_selector, 10)                   # 수동 스테이지 선택 (1,2,3)
        
        # 미션 모드 및 배달 관련 구독
        self.create_subscription(ModeState, '/mode_state', self._on_mode_state, 10)                       # 주차/배달 모드 선택
        self.create_subscription(PoseArray, '/deliverysign_spot', self._on_delivery_spots, 10)            # 배달 표지판 후보 위치들
        self.create_subscription(Int32, '/target_sign', self._on_target_sign, 10)                         # 목표 표지판 ID (3,4,5 -> B1,B2,B3)
        self.create_subscription(PoseStamped, '/delivery_pick_target', self._on_delivery_pick_target, 10)  # 상차 목표 위치 (map 프레임)
        self.create_subscription(PoseStamped, '/delivery_drop_target', self._on_delivery_drop_target, 10)  # 하차 목표 위치 (map 프레임)

        # ==================== 타이머 설정 ====================
        self.create_timer(0.2, self._on_timer)  # 0.2초마다 스테이지 재평가 및 경로 발행

        self.get_logger().info('Planner node started (Stage-1 goal computation).')

    # ==================== 콜백 함수들 ====================
    def _on_parking_area_idx(self, msg: Int32) -> None:
        """주차 공간 인덱스 수신 콜백"""
        self._parking_area_idx = int(msg.data)
        self.get_logger().info(f'주차 공간 인덱스 수신: {self._parking_area_idx}')
        self._maybe_publish_goal()

    def _on_parking_pose(self, msg: PoseStamped) -> None:
        """주차 공간 위치 수신 콜백"""
        self._parking_pose = msg
        self.get_logger().info(f'주차 공간 위치 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self._maybe_publish_goal()

    def _on_stable_cones(self, msg: ObstacleArray) -> None:
        """안정화된 콘 정보 수신 콜백 (충돌 회피용)"""
        self._stable_cones = msg.obstacles
        self.get_logger().info(f'안정화된 콘 {len(self._stable_cones)}개 수신')
        self._maybe_publish_goal()


    def _on_odom(self, msg: Odometry) -> None:
        """차량 오도메트리 수신 콜백 (차량 현재 위치)"""
        self._odom = msg
        # 스테이지 전환 체크를 위해 _maybe_publish_goal 호출
        self._maybe_publish_goal()

    def _on_stage_selector(self, msg: Int32) -> None:
        """수동 스테이지 선택 콜백 (1, 2, 3)"""
        val = int(msg.data)
        if val in (1, 2, 3):
            self._selector_stage = val
            self._stage = val
            self.get_logger().info(f'스테이지 수동 선택: {self._stage}')
            self._maybe_publish_goal()

    def _on_mode_state(self, msg: ModeState) -> None:
        """미션 모드 상태 수신 콜백 (주차/배달 모드 전환)"""
        try:
            if int(msg.current_mode) == int(ModeState.DELIVERY):
                if self._mission != 'delivery':
                    self.get_logger().info('미션 모드 전환: 배달 모드')
                self._mission = 'delivery'
            elif int(msg.current_mode) == int(ModeState.PARKING):
                if self._mission != 'parking':
                    self.get_logger().info('미션 모드 전환: 주차 모드')
                self._mission = 'parking'
        except Exception:
            pass

    def _on_delivery_spots(self, msg: PoseArray) -> None:
        """배달 표지판 후보 위치들 수신 콜백
        - 센서퓨전에서 수신한 표지판 후보 좌표 배열
        - frame_id를 map으로 강제하여 일관성 유지
        - delivery_auto_activate=True일 경우, 이 토픽 수신만으로도 배달 미션으로 전환
        - enable_delivery_planning=False면 실제 경로 생성은 수행하지 않음
        """
        # 일관성을 위해 헤더를 map 프레임으로 강제 설정
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        self._delivery_spots = msg
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('배달 미션 자동 활성화 (표지판 위치 수신)')

    def _on_target_sign(self, msg: Int32) -> None:
        """목표 표지판 ID 수신 콜백 (3=B1, 4=B2, 5=B3)
        - 모드 셀렉터/인지 노드에서 목표 B 표지판 식별자를 수신
        - delivery_auto_activate=True일 때 배달 미션으로 자동 전환만 수행
        - 실제 목표 위치는 pick/drop 타겟 토픽 또는 spots로 보완됨
        """
        try:
            self._delivery_target_sign = int(msg.data)
        except Exception:
            self._delivery_target_sign = None
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('배달 미션 자동 활성화 (목표 표지판 수신)')

    def _on_delivery_pick_target(self, msg: PoseStamped) -> None:
        """상차 목표 위치 수신 콜백
        - 모드 셀렉터가 산출한 상차 타겟 표지판 위치
        - 항상 map 프레임으로 강제 저장
        - delivery_auto_activate=True면 이 토픽만으로 배달 미션 활성화
        """
        # map 프레임으로 강제 설정
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        self._delivery_pick_target = msg
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('배달 미션 자동 활성화 (상차 목표 수신)')

    def _on_delivery_drop_target(self, msg: PoseStamped) -> None:
        """하차 목표 위치 수신 콜백
        - 모드 셀렉터가 산출한 하차 타겟 표지판 위치
        - 항상 map 프레임으로 강제 저장
        - delivery_auto_activate=True면 이 토픽만으로 배달 미션 활성화
        """
        # map 프레임으로 강제 설정
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        self._delivery_drop_target = msg
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('배달 미션 자동 활성화 (하차 목표 수신)')

    # ==================== 핵심 로직 ====================
    def _on_timer(self) -> None:
        """주기적으로 호출되는 메인 루프 (0.2초마다)
        - enable_delivery_planning=True이고 미션이 delivery일 때만 배달 경로 생성
        - 그 외에는 주차 경로 생성 로직 수행
        """
        if bool(self.get_parameter('enable_delivery_planning').value) and self._mission == 'delivery':
            self._maybe_publish_delivery()
        else:
            self._maybe_publish_goal()
    def _get_slot_dimensions_from_area_idx(self, area_idx: int) -> Tuple[float, float]:
        """주차 공간 인덱스에 따른 크기 반환 (현재는 모든 슬롯 동일)
        Args:
            area_idx: 주차 공간 인덱스
        Returns:
            (길이, 폭) 튜플 [m]
        """
        return 5.0, 2.5  # 모든 슬롯 동일: 길이 5m, 폭 2.5m

    def _transform_odom_to_map(self, odom: Odometry) -> Optional[PoseStamped]:
        """오도메트리를 map 프레임으로 변환
        Args:
            odom: 오도메트리 메시지
        Returns:
            map 프레임의 PoseStamped 또는 None (변환 실패 시)
        """
        try:
            # 오도메트리를 PoseStamped로 변환
            pose_stamped = PoseStamped()
            pose_stamped.header = odom.header
            pose_stamped.pose = odom.pose.pose
            
            # map 프레임으로 변환
            transformed_pose = self._tf_buffer.transform(pose_stamped, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            return transformed_pose
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
            return None

    def _min_distance_to_cones(self, x: float, y: float) -> float:
        """콘까지의 최소 거리 계산 (충돌 회피용)
        Args:
            x, y: 검사할 위치 좌표
        Returns:
            가장 가까운 콘까지의 거리 [m] (콘이 없으면 무한대)
        """
        if not self._stable_cones:
            return float('inf')
        
        min_dist = float('inf')
        for cone in self._stable_cones:
            dist = math.hypot(x - cone.center.x, y - cone.center.y) - cone.radius
            min_dist = min(min_dist, dist)
        
        return min_dist

    def _compute_stage1_goal(self, parking_pose: PoseStamped) -> Tuple[PoseStamped, MarkerArray]:
        """Stage-1 목표 위치 계산 (주차 공간 앞으로 전진 이동)
        Args:
            parking_pose: 주차 공간 위치
        Returns:
            (목표 위치, 시각화 마커) 튜플
        """
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        center = (parking_pose.pose.position.x, parking_pose.pose.position.y)

        # 주차 공간 크기 결정
        if self._parking_area_idx is not None:
            L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
        else:
            L, W = 5.0, 2.5  # 기본값

        # Stage-1 목표 계산 파라미터
        # 목표: 주차 공간 전방 경계선까지 올라간 뒤 정지
        # 전방 이동량 = L/2 (센터→전방 경계) + front_margin
        front_margin = float(self.get_parameter('front_margin').value)
        s_along = (L / 2.0) + front_margin
        clear_lat = float(self.get_parameter('clear_lateral').value)
        yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))

        # 주차 공간 기준 단위 벡터 계산
        u_long = (math.cos(yaw_slot), math.sin(yaw_slot))      # 주차 공간 진행 방향
        u_lat = (-math.sin(yaw_slot), math.cos(yaw_slot))      # 주차 공간 측면 방향 (도로쪽이 +u_lat)

        # 측면 배치: 주차 공간 기준 오프셋
        lateral = (W / 2.0 + clear_lat)

        # Stage-1 정지 위치 계산
        x_entry = center[0] + s_along * u_long[0] + lateral * u_lat[0]
        y_entry = center[1] + s_along * u_long[1] + lateral * u_lat[1]
        yaw_entry = wrap_to_pi(yaw_slot + yaw_offset)

        # 목표 위치 메시지 생성
        goal = PoseStamped()
        goal.header = parking_pose.header
        goal.pose.position.x = x_entry
        goal.pose.position.y = y_entry
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw_entry)

        # 시각화 마커 생성
        markers = MarkerArray()

        # 마커 0: Stage-1 목표 위치 화살표
        arrow = Marker()
        arrow.header = goal.header
        arrow.ns = 'stage1_goal'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = goal.pose
        arrow.scale.x = 1.0  # 화살표 길이
        arrow.scale.y = 0.2
        arrow.scale.z = 0.2
        arrow.color.a = 1.0
        arrow.color.r = 0.1
        arrow.color.g = 1.0
        arrow.color.b = 0.1
        markers.markers.append(arrow)

        # 마커 1: 주차 공간 중심 구체
        center_mk = Marker()
        center_mk.header = goal.header
        center_mk.ns = 'stage1_goal'
        center_mk.id = 1
        center_mk.type = Marker.SPHERE
        center_mk.action = Marker.ADD
        center_mk.pose.position.x = center[0]
        center_mk.pose.position.y = center[1]
        center_mk.pose.position.z = 0.0
        center_mk.scale.x = 0.25
        center_mk.scale.y = 0.25
        center_mk.scale.z = 0.25
        center_mk.color.a = 0.9
        center_mk.color.r = 0.2
        center_mk.color.g = 0.6
        center_mk.color.b = 1.0
        markers.markers.append(center_mk)

        # 마커 2: 중심에서 목표까지의 연결선
        line = Marker()
        line.header = goal.header
        line.ns = 'stage1_goal'
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.a = 0.9
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 0.0
        p0 = Point(); p0.x = center[0]; p0.y = center[1]; p0.z = 0.0
        p1 = Point(); p1.x = x_entry; p1.y = y_entry; p1.z = 0.0
        line.points.append(p0)
        line.points.append(p1)
        markers.markers.append(line)

        return goal, markers

    def _check_stage_advancement(self) -> bool:
        """차량이 현재 스테이지 목표에 도달했는지 확인하고 스테이지 전환
        Returns:
            스테이지가 전환되었으면 True, 아니면 False
        """
        # auto_advance가 비활성화되어 있으면 자동 전환하지 않음
        if not bool(self.get_parameter('auto_advance').value):
            return False
            
        if self._odom is None:
            return False
        
        # UTM 좌표를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(self._odom)
        if transformed_pose is None:
            self.get_logger().warn("스테이지 전환 체크: TF 변환 실패")
            return False
        
        current_pos = transformed_pose.pose.position
        tolerance = float(self.get_parameter('stage_position_tolerance').value)
        
        old_stage = self._stage
        
        # Stage-1에서 Stage-2로 전환 (전진 → 후진)
        if self._stage == 1 and self._last_stage1_goal is not None:
            distance = math.hypot(
                current_pos.x - self._last_stage1_goal.pose.position.x,
                current_pos.y - self._last_stage1_goal.pose.position.y
            )
            if distance < tolerance:
                self._stage = 2
                self.get_logger().info(f"스테이지 전환: 1 → 2 (거리={distance:.2f}m < {tolerance}m)")
                
                # Stage-1 도착 시 후진 모드 활성화
                self._publish_reverse_flag(True)
        
        # Stage-2에서 Stage-3으로 전환 (후진 → 전진)
        elif self._stage == 2 and self._last_stage2_goal is not None:
            distance = math.hypot(
                current_pos.x - self._last_stage2_goal.pose.position.x,
                current_pos.y - self._last_stage2_goal.pose.position.y
            )
            if distance < tolerance:
                # Stage-3 활성화 여부 확인
                if bool(self.get_parameter('stage3_enabled').value):
                    self._stage = 3
                    self.get_logger().info(f"스테이지 전환: 2 → 3 (거리={distance:.2f}m < {tolerance}m)")
                    
                    # Stage-3에서는 전진 모드 (최종 정렬)
                    self._publish_reverse_flag(False)
                else:
                    self.get_logger().info(f"Stage-3 비활성화됨 - 주차 완료 (거리={distance:.2f}m < {tolerance}m)")
                    # Stage-3가 비활성화된 경우 주차 완료로 처리
                    self.get_logger().info("주차 미션 완료 (Stage-3 생략)")
        
        return self._stage != old_stage

    def _publish_reverse_flag(self, reverse: bool) -> None:
        """후진 모드 플래그 발행
        Args:
            reverse: True면 후진 모드, False면 전진 모드
        """
        reverse_msg = Bool()
        reverse_msg.data = reverse
        self._reverse_flag_pub.publish(reverse_msg)
        mode = "후진" if reverse else "전진"
        self.get_logger().info(f"후진 모드 플래그 발행: {mode}")

    def _check_parking_completion(self) -> bool:
        """주차 완료 조건 확인 및 3초 정지 시작
        Returns:
            주차 완료가 감지되었으면 True, 아니면 False
        """
        if self._parking_complete_triggered:
            return False  # 이미 트리거됨
            
        if self._odom is None or self._parking_pose is None:
            return False
            
        # UTM 좌표를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(self._odom)
        if transformed_pose is None:
            return False
            
        # 방향 임계치 확인
        yaw_tolerance = math.radians(float(self.get_parameter('parking_complete_yaw_tolerance_deg').value))
        current_yaw = quaternion_to_yaw(transformed_pose.pose.orientation)
        parking_yaw = quaternion_to_yaw(self._parking_pose.pose.orientation)
        yaw_diff = abs(wrap_to_pi(current_yaw - parking_yaw))
        
        self.get_logger().info(f"주차 완료 체크 - 현재 방향: {math.degrees(current_yaw):.1f}°, 주차 방향: {math.degrees(parking_yaw):.1f}°, 차이: {math.degrees(yaw_diff):.1f}°")
        
        # Stage-3 활성화 여부에 따른 분기
        stage3_enabled = bool(self.get_parameter('stage3_enabled').value)
        
        if stage3_enabled:
            # Stage-3가 활성화된 경우: Stage-3 골에 도달했는지 확인
            if self._stage == 3 and self._last_stage3_goal is not None:
                pos_tol = float(self.get_parameter('stage_position_tolerance').value)
                if self._is_near_pose(transformed_pose, self._last_stage3_goal, pos_tol, yaw_tolerance):
                    self.get_logger().info("🎯 Stage-3 골 도달 - 주차 완료 조건 만족!")
                    return True
        else:
            # Stage-3가 비활성화된 경우: Stage-2에서 주차 위치 도달 확인
            if self._stage == 2 and self._last_stage2_goal is not None:
                pos_tol = float(self.get_parameter('stage_position_tolerance').value)
                if self._is_near_pose(transformed_pose, self._last_stage2_goal, pos_tol, yaw_tolerance):
                    self.get_logger().info("🎯 Stage-2에서 주차 위치 도달 - 주차 완료 조건 만족!")
                    return True
                    
        return False

    def _start_parking_stop_timer(self) -> None:
        """주차 완료 3초 정지 타이머 시작"""
        if self._parking_complete_state != 'normal':
            return  # 이미 진행 중
            
        self._parking_complete_state = 'stopping'
        self._parking_stop_timer = self.get_clock().now().nanoseconds / 1e9  # 현재 시간 (초)
        self._parking_complete_triggered = True
        
        # 3초 정지 플래그 발행
        stop_msg = Bool()
        stop_msg.data = True
        self._parking_stop_flag_pub.publish(stop_msg)
        
        stop_duration = float(self.get_parameter('parking_stop_duration').value)
        self.get_logger().info(f"🛑 주차 완료 감지 - {stop_duration}초 정지 시작")

    def _check_parking_stop_timer(self) -> None:
        """주차 정지 타이머 확인 및 완료 처리"""
        if self._parking_complete_state != 'stopping' or self._parking_stop_timer is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self._parking_stop_timer
        stop_duration = float(self.get_parameter('parking_stop_duration').value)
        
        if elapsed_time >= stop_duration:
            # 3초 경과 - 주차 종료 플래그 발행
            self._parking_complete_state = 'completed'
            
            # 주차 종료 플래그 발행
            complete_msg = Bool()
            complete_msg.data = True
            self._parking_complete_flag_pub.publish(complete_msg)
            
            # 정지 플래그 해제
            stop_msg = Bool()
            stop_msg.data = False
            self._parking_stop_flag_pub.publish(stop_msg)
            
            self.get_logger().info("✅ 주차 미션 완료 - 주차 종료 플래그 발행")

    def _maybe_publish_goal(self) -> None:
        """현재 스테이지의 경로를 계산하고 통합 waypoint로 발행
        - 주차 관련 정보가 모두 준비되면 경로 생성
        - 현재 스테이지에 맞는 경로만 계산하여 발행
        """
        # 필수 입력 정보 확인
        if self._parking_pose is None:
            self.get_logger().warn("주차 공간 위치 정보 없음, 경로 생성 중단")
            return
            
        if self._odom is None:
            self.get_logger().warn("차량 위치 정보 없음, 경로 생성 중단")
            return
            
        # 오도메트리 정보를 로컬 변수로 복사 (변경 방지)
        current_odom = self._odom
        self.get_logger().info("경로 생성 시작")
        self.get_logger().info(f"현재 차량 위치: ({current_odom.pose.pose.position.x:.2f}, {current_odom.pose.pose.position.y:.2f})")

        # Stage-1 목표와 시각화 마커는 항상 갱신 (디버그용)
        try:
            stage1_goal, markers = self._compute_stage1_goal(self._parking_pose)
            self._last_stage1_goal = stage1_goal
            self._goal_pub.publish(stage1_goal)
            self._vis_pub.publish(markers)
        except Exception as e:
            self.get_logger().warn(f'Stage-1 목표 계산 실패: {e}')
            return

        # 테스트 모드: 주차 포즈 수신 시 즉시 S자 경로 생성
        test_mode = bool(self.get_parameter('test_mode_immediate_s_curve').value)
        if test_mode:
            self.get_logger().info("테스트 모드 활성화 - 즉시 S자 경로 생성")
            self._stage = 2  # 강제로 Stage-2로 설정
            self._publish_reverse_flag(True)  # 후진 모드 활성화
        else:
            # 자동 스테이지 전환 체크
            stage_advanced = self._check_stage_advancement()
            if stage_advanced:
                self.get_logger().info(f"스테이지 전환됨: {self._stage}")
        
        # 현재 스테이지에 맞는 경로만 계산하고 발행
        current_path = None
        
        if self._stage == 1:
            # Stage-1 경로 (전진 이동)
            show_stage1 = bool(self.get_parameter('show_stage1_path').value)
            if show_stage1:
                self.get_logger().info("Stage-1 경로 계산 중...")
                try:
                    current_path = self._compute_stage1_path(current_odom, stage1_goal)
                    if current_path is not None:
                        self._path_pub.publish(current_path)
                        self._publish_points(self._stage1_points_pub, current_path)
                        self.get_logger().info(f"Stage-1 경로 생성: {len(current_path.poses)}개 점")
                    else:
                        self.get_logger().warn("Stage-1 경로 생성 실패")
                except Exception as e:
                    self.get_logger().error(f"Stage-1 경로 계산 실패: {e}")
                    import traceback
                    self.get_logger().error(f"상세 오류: {traceback.format_exc()}")
        
        elif self._stage == 2:
            # Stage-2 목표와 경로 (후진 주차)
            self.get_logger().info("=== Stage-2 디버그 시작 ===")
            self.get_logger().info(f"현재 차량 위치 (odom): ({current_odom.pose.pose.position.x:.2f}, {current_odom.pose.pose.position.y:.2f})")
            self.get_logger().info(f"주차 공간 위치: ({self._parking_pose.pose.position.x:.2f}, {self._parking_pose.pose.position.y:.2f})")
            self.get_logger().info(f"주차 공간 인덱스: {self._parking_area_idx}")
            
            # Stage-2 목표 계산
            self.get_logger().info("Stage-2 목표 계산 중...")
            stage2_goal = self._compute_stage2_goal(self._parking_pose)
            if stage2_goal is not None:
                self._stage2_goal_pub.publish(stage2_goal)
                self.get_logger().info(f"✅ Stage-2 목표 계산 성공: ({stage2_goal.pose.position.x:.2f}, {stage2_goal.pose.position.y:.2f})")
                
                # Stage-2 경로 계산 (벽 따라 S-curve 우선, 기존 S-curve 폴백)
                s_curve_enabled = bool(self.get_parameter('s_curve_enabled').value)
                self.get_logger().info(f"📋 Stage-2 파라미터 - S자경로: {s_curve_enabled}")
                
                current_path = None
                
                # 0) 작년 로직 경로 최우선
                if bool(self.get_parameter('oldlogic_use').value):
                    self.get_logger().info("🔄 Stage-2: using OLD-LOGIC arc+arc path between Stage1→Stage2")
                    # Stage-1 goal은 self._last_stage1_goal 에 이미 저장/발행됨
                    if self._last_stage1_goal is not None:
                        current_path = self._compute_stage2_path_oldlogic(self._last_stage1_goal,
                                                                          stage2_goal,
                                                                          self._parking_pose)
                        if current_path is not None:
                            self.get_logger().info(f"✅ Old-logic path OK: {len(current_path.poses)} pts")
                        else:
                            self.get_logger().warn("⚠️ 작년 로직 경로 생성 실패, 하드코딩 경로로 폴백")
                    else:
                        self.get_logger().warn("⚠️ Stage-1 목표가 없음, 하드코딩 경로로 폴백")
                
                # 1) 하드코딩 경로 (폴백)
                if current_path is None and bool(self.get_parameter('stage2_use_hardcoded').value):
                    self.get_logger().info("🔄 Stage-2 하드코딩 S-curve 경로 생성 시도")
                    try:
                        current_path = self._compute_stage2_path_hardcoded(current_odom, self._parking_pose)
                        if current_path is not None:
                            self.get_logger().info(f"✅ Stage-2 HARDCODED path: {len(current_path.poses)} pts")
                        else:
                            self.get_logger().warn("⚠️ 하드코딩 경로 생성 실패, 벽 따라 S-curve로 폴백")
                    except Exception as e:
                        self.get_logger().warn(f"⚠️ 하드코딩 경로 예외 발생: {e}, 벽 따라 S-curve로 폴백")
                
                if s_curve_enabled and current_path is None:
                    # 1순위: 벽 따라 S-curve 경로 (그림 같은 형태)
                    self.get_logger().info("🔄 Stage-2 벽 따라 S-curve 경로 생성 시도")
                    try:
                        current_path = self._stage2_path_shape_wallS(current_odom, self._parking_pose)
                        if current_path is not None:
                            self.get_logger().info(f"✅ Stage-2 벽 따라 S-curve 경로 생성 성공: {len(current_path.poses)}개 점")
                        else:
                            self.get_logger().warn("⚠️ 벽 따라 S-curve 경로 생성 실패, 기존 S-curve로 폴백")
                    except Exception as e:
                        self.get_logger().warn(f"⚠️ 벽 따라 S-curve 예외 발생: {e}, 기존 S-curve로 폴백")
                    
                    # 2순위: 기존 S자 경로 (폴백)
                    if current_path is None:
                        self.get_logger().info("🔄 Stage-2 기존 S자 경로 생성 시도")
                        current_path = self._compute_stage2_path_s_curve(current_odom, self._parking_pose)
                        if current_path is not None:
                            self.get_logger().info(f"✅ Stage-2 기존 S자 경로 생성 성공: {len(current_path.poses)}개 점")
                        else:
                            self.get_logger().error("❌ 기존 S자 경로도 생성 실패")
                else:
                    self.get_logger().error("❌ S자 경로가 비활성화됨 - Stage-2 경로 생성 불가")
                
                if current_path is not None:
                    self._stage2_path_pub.publish(current_path)
                    self._publish_points(self._stage2_points_pub, current_path)
                    self.get_logger().info(f"📤 Stage-2 경로 발행 완료: {len(current_path.poses)}개 점")
                else:
                    self.get_logger().error("❌ Stage-2 모든 경로 생성 실패")
            else:
                self.get_logger().error("❌ Stage-2 목표 계산 실패")
            self.get_logger().info("=== Stage-2 디버그 종료 ===")
        
        elif self._stage == 3:
            # Stage-3 활성화 여부 확인
            if bool(self.get_parameter('stage3_enabled').value):
                # Stage-3 목표와 경로 (최종 정렬)
                stage3_goal = self._compute_stage3_goal(self._parking_pose)
                if stage3_goal is not None:
                    self._stage3_goal_pub.publish(stage3_goal)
                    self.get_logger().info(f"Stage-3 목표 발행: ({stage3_goal.pose.position.x:.2f}, {stage3_goal.pose.position.y:.2f})")
                    yaw_slot = quaternion_to_yaw(self._parking_pose.pose.orientation)
                    current_path = self._compute_stage3_path(current_odom, yaw_slot)
                    
                    if current_path is not None:
                        self._stage3_path_pub.publish(current_path)
                        self._publish_points(self._stage3_points_pub, current_path)
                        self.get_logger().info(f"Stage-3 경로 생성: {len(current_path.poses)}개 점")
                else:
                    self.get_logger().warn("Stage-3 목표 계산 실패")
            else:
                self.get_logger().warn("Stage-3가 비활성화되어 있음 - 경로 생성 중단")
        
        # 현재 스테이지 경로를 메인 waypoints 토픽으로 발행 (방향 정보 제거)
        if current_path is not None:
            # 방향 정보를 제거한 깨끗한 경로 생성 (제어 파트용)
            clean_path = self._remove_orientation_from_path(current_path)
            self._waypoints_pub.publish(clean_path)
            self._publish_points(self._waypoints_points_pub, clean_path)
            self.get_logger().info(f"스테이지 {self._stage} 웨이포인트 발행: {len(clean_path.poses)}개 점 (방향 정보 제거됨)")
        else:
            self.get_logger().warn(f"스테이지 {self._stage}에 사용 가능한 경로 없음")
        
        # 주차 완료 체크 및 처리
        if self._parking_complete_state == 'normal':
            # 주차 완료 조건 확인
            if self._check_parking_completion():
                self._start_parking_stop_timer()
        elif self._parking_complete_state == 'stopping':
            # 3초 정지 타이머 확인
            self._check_parking_stop_timer()
        
        # 디버그: 모든 스테이지 목표와 경로도 발행 (시각화용)
        if bool(self.get_parameter('publish_debug_paths').value):
            self._publish_all_debug_paths(current_odom)
    
    def _publish_all_debug_paths(self, current_odom: Odometry) -> None:
        """모든 스테이지의 목표와 경로를 디버그 토픽으로 발행 (시각화용)
        Args:
            current_odom: 현재 차량 오도메트리
        """
        try:
            # Stage-1 목표와 경로 (항상 발행)
            if self._parking_pose is not None:
                stage1_goal, _ = self._compute_stage1_goal(self._parking_pose)
                if stage1_goal is not None:
                    self._goal_pub.publish(stage1_goal)
                    self._last_stage1_goal = stage1_goal
                
                show_stage1 = bool(self.get_parameter('show_stage1_path').value)
                if show_stage1:
                    stage1_path = self._compute_stage1_path(current_odom, stage1_goal)
                    if stage1_path is not None:
                        self._path_pub.publish(stage1_path)
                        self._publish_points(self._stage1_points_pub, stage1_path)
            
            # Stage-2 목표와 경로
            if self._parking_pose is not None:
                stage2_goal = self._compute_stage2_goal(self._parking_pose)
                if stage2_goal is not None:
                    self._stage2_goal_pub.publish(stage2_goal)
                    self._last_stage2_goal = stage2_goal
                    
                    # 작년 로직 경로 우선
                    if bool(self.get_parameter('oldlogic_use').value) and self._last_stage1_goal is not None:
                        stage2_path = self._compute_stage2_path_oldlogic(self._last_stage1_goal, stage2_goal, self._parking_pose)
                    elif bool(self.get_parameter('s_curve_enabled').value):
                        stage2_path = self._compute_stage2_path_s_curve(current_odom, self._parking_pose)
                    
                    if stage2_path is not None:
                        self._stage2_path_pub.publish(stage2_path)
                        self._publish_points(self._stage2_points_pub, stage2_path)
                
                # Stage-3 목표와 경로 (활성화된 경우에만)
                if bool(self.get_parameter('stage3_enabled').value):
                    stage3_goal = self._compute_stage3_goal(self._parking_pose)
                    if stage3_goal is not None:
                        self._stage3_goal_pub.publish(stage3_goal)
                        self._last_stage3_goal = stage3_goal
                        
                        yaw_slot = quaternion_to_yaw(self._parking_pose.pose.orientation)
                        stage3_path = self._compute_stage3_path(current_odom, yaw_slot)
                        if stage3_path is not None:
                            self._stage3_path_pub.publish(stage3_path)
                            self._publish_points(self._stage3_points_pub, stage3_path)
        except Exception as e:
            self.get_logger().warn(f"디버그 경로 발행 실패: {e}")

    # ==================== 배달 미션 ====================
    def _maybe_publish_delivery(self) -> None:
        """배달 미션 경로 발행 (상차 → 하차 단계)
        - 상차 단계: pick 타겟(있으면 우선)을 사용, 없으면 spots에서 진행방향 앞 최근접 선택
        - 하차 단계: drop 타겟(있으면 우선)을 사용, 없으면 spots에서 진행방향 앞 최근접 선택
        - 각 단계에서 표지판 앞 delivery_stop_offset [m] 위치에 정지 목표 생성 후 직선 경로 샘플링
        - 완료 판정: 상차는 오프셋 목표 도달 시 dropoff로 전환, 하차는 target_sign이 설정된 뒤 오프셋 목표 도달 시 완료
        주의: enable_delivery_planning=False면 이 함수는 호출되지 않음
        """
        start_pose = self._get_start_pose()
        if start_pose is None:
            return
        publish_unified = bool(self.get_parameter('publish_unified_waypoints').value)
        pos_tol = float(self.get_parameter('delivery_position_tolerance').value)
        yaw_tol = math.radians(float(self.get_parameter('delivery_yaw_tolerance_deg').value))

        goal: Optional[PoseStamped] = None
        # Prefer explicit targets from selector, else fall back to fused spots
        if self._delivery_phase == 'pickup' and self._delivery_pick_target is not None:
            goal = self._delivery_pick_target
        elif self._delivery_phase == 'dropoff' and self._delivery_drop_target is not None:
            goal = self._delivery_drop_target
        elif self._delivery_spots is not None and self._delivery_spots.poses:
            goal = self._select_nearest_ahead_spot(start_pose, self._delivery_spots)

        stop_offset = float(self.get_parameter('delivery_stop_offset').value)

        if self._delivery_phase == 'pickup':
            if goal is not None:
                goal_off = self._compute_offset_goal(start_pose, goal, stop_offset)
                path = self._compute_straight_path(start_pose, goal_off)
                if path is not None and path.poses:
                    self._delivery_pick_path_pub.publish(path)
                    self._publish_points(self._delivery_pick_points_pub, path)
                    if publish_unified:
                        self._waypoints_pub.publish(path)
                        self._publish_points(self._waypoints_points_pub, path)
                if self._is_near_pose(start_pose, goal_off, pos_tol, yaw_tol):
                    self._delivery_phase = 'dropoff'
                    self.get_logger().info('Delivery: pickup reached -> dropoff phase')
                    return
        else:
            if goal is not None:
                goal_off = self._compute_offset_goal(start_pose, goal, stop_offset)
                path = self._compute_straight_path(start_pose, goal_off)
                if path is not None and path.poses:
                    self._delivery_drop_path_pub.publish(path)
                    self._publish_points(self._delivery_drop_points_pub, path)
                    if publish_unified:
                        self._waypoints_pub.publish(path)
                        self._publish_points(self._waypoints_points_pub, path)
                # 종료 조건: B 표지판(타겟) 인지 이후 도달
                if self._delivery_target_sign is not None and self._is_near_pose(start_pose, goal_off, pos_tol, yaw_tol):
                    self.get_logger().info('Delivery: dropoff reached after target sign (mission done)')

    def _select_nearest_ahead_spot(self, start_pose: PoseStamped, spots: PoseArray) -> Optional[PoseStamped]:
        """진행 방향(현재 yaw) 앞쪽에 있는 후보들 중 최근접 표지판 위치를 선택.
        - 앞쪽 후보가 없으면 전체 최근접으로 대체.
        반환: PoseStamped(map 프레임), 헤딩은 현재 헤딩 유지.
        """
        if not spots.poses:
            return None
        sx = float(start_pose.pose.pose.position.x)
        sy = float(start_pose.pose.pose.position.y)
        yaw = quaternion_to_yaw(start_pose.pose.pose.orientation)
        fx = math.cos(yaw); fy = math.sin(yaw)
        best = None
        best_dist = None
        for p in spots.poses:
            gx = float(p.position.x)
            gy = float(p.position.y)
            dx = gx - sx; dy = gy - sy
            ahead = (dx * fx + dy * fy) > 0.0
            if not ahead:
                continue
            d = math.hypot(dx, dy)
            if best is None or d < best_dist:
                best = (gx, gy)
                best_dist = d
        if best is None:
            for p in spots.poses:
                gx = float(p.position.x); gy = float(p.position.y)
                d = math.hypot(gx - sx, gy - sy)
                if best is None or d < best_dist:
                    best = (gx, gy); best_dist = d
        if best is None:
            return None
        g = PoseStamped()
        g.header = start_pose.header
        g.header.frame_id = str(self.get_parameter('frame_id').value)
        g.pose.position.x = best[0]
        g.pose.position.y = best[1]
        g.pose.position.z = 0.0
        g.pose.orientation = yaw_to_quaternion(yaw)
        return g

    def _compute_straight_path(self, start_pose: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """start→goal 직선 구간을 일정 간격(path_resolution)으로 샘플링하여 Path 생성.
        - 포즈 방향은 순간 방향(atan2(dy,dx))으로 설정.
        """
        ds = float(self.get_parameter('path_resolution').value)
        x0 = float(start_pose.pose.pose.position.x)
        y0 = float(start_pose.pose.pose.position.y)
        x1 = float(goal.pose.position.x)
        y1 = float(goal.pose.position.y)
        dx = x1 - x0; dy = y1 - y0
        L = math.hypot(dx, dy)
        n = max(1, int(L / max(ds, 1e-3)))
        path = Path(); path.header = goal.header
        for i in range(n + 1):
            t = i / max(1, n)
            px = x0 + t * dx
            py = y0 + t * dy
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            yaw = math.atan2(dy, dx) if L > 1e-6 else quaternion_to_yaw(start_pose.pose.pose.orientation)
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        return path

    def _compute_offset_goal(self, start_pose: PoseStamped, sign_pose: PoseStamped, offset_m: float) -> PoseStamped:
        """표지판(sign_pose)까지 직진한다고 가정하고, 표지판 앞 offset_m 지점에 정지 목표 생성.
        - 목표 헤딩은 start→sign 방향을 사용.
        - 헤더 프레임은 map으로 유지.
        """
        sx = float(start_pose.pose.pose.position.x)
        sy = float(start_pose.pose.pose.position.y)
        gx = float(sign_pose.pose.position.x)
        gy = float(sign_pose.pose.position.y)
        dx = gx - sx
        dy = gy - sy
        L = math.hypot(dx, dy)
        if L < 1e-6:
            # 방향 정보를 잃은 경우: 현재 자세를 유지
            return sign_pose
        ux = dx / L; uy = dy / L
        goal = PoseStamped()
        goal.header = sign_pose.header
        goal.pose.position.x = gx - offset_m * ux
        goal.pose.position.y = gy - offset_m * uy
        goal.pose.position.z = 0.0
        yaw = math.atan2(dy, dx)
        goal.pose.orientation = yaw_to_quaternion(yaw)
        return goal

    def _get_start_pose(self) -> Optional[PoseStamped]:
        """현재 차량 위치를 map 프레임으로 변환하여 반환
        Returns:
            map 프레임의 PoseStamped 또는 None (변환 실패 시)
        """
        if self._odom is not None:
            # UTM 좌표를 map 프레임으로 변환
            transformed_pose = self._transform_odom_to_map(self._odom)
            if transformed_pose is not None:
                return transformed_pose
            else:
                self.get_logger().warn("_get_start_pose: TF 변환 실패")
        return None

    def _remove_orientation_from_path(self, path: Path) -> Path:
        """Path에서 방향 정보를 제거하여 반환 (제어 파트용)
        Args:
            path: 원본 Path 메시지
        Returns:
            방향 정보가 제거된 Path 메시지
        """
        clean_path = Path()
        clean_path.header = path.header
        for p in path.poses:
            clean_pose = PoseStamped()
            clean_pose.header = p.header
            clean_pose.pose.position = p.pose.position
            # 방향 정보 제거 - 제어 파트에서 원하지 않음
            clean_pose.pose.orientation.w = 1.0  # 기본 쿼터니언 (방향 정보 없음)
            clean_pose.pose.orientation.x = 0.0
            clean_pose.pose.orientation.y = 0.0
            clean_pose.pose.orientation.z = 0.0
            clean_path.poses.append(clean_pose)
        return clean_path

    def _publish_points(self, pub, path: Path) -> None:
        """Path를 PoseArray로 변환하여 발행 (방향 정보 제거, 위치만 포함)
        Args:
            pub: PoseArray 퍼블리셔
            path: 발행할 Path 메시지
        """
        arr = PoseArray()
        arr.header = path.header
        for p in path.poses:
            pose = Pose()
            pose.position = p.pose.position
            # 방향 정보 제거 - 제어 파트에서 원하지 않음
            # pose.orientation = p.pose.orientation  # 제거됨
            pose.orientation.w = 1.0  # 기본 쿼터니언 (방향 정보 없음)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            arr.poses.append(pose)
        pub.publish(arr)

    def _evaluate_and_advance_stage(self) -> bool:
        """오돔 기반으로 현재 스테이지 완료를 평가하고 필요 시 다음 스테이지로 전환.
        Returns True if stage was advanced."""
        if self._odom is None or self._parking_pose is None:
            return False
        
        # UTM 좌표를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(self._odom)
        if transformed_pose is None:
            self.get_logger().warn("스테이지 평가: TF 변환 실패")
            return False
        
        pos_tol = float(self.get_parameter('stage_position_tolerance').value)
        yaw_tol = math.radians(float(self.get_parameter('stage_yaw_tolerance_deg').value))

        if self._stage == 1 and self._last_stage1_goal is not None:
            # 변환된 pose 사용
            if self._is_near_pose(transformed_pose, self._last_stage1_goal, pos_tol, yaw_tol):
                self._stage = 2
                self.get_logger().info('Stage-1 complete -> Stage-2')
                # Stage-1 도착 시 후진 모드 활성화
                self._publish_reverse_flag(True)
                return True

        elif self._stage == 2:
            if self._last_stage2_goal is not None:
                # 변환된 pose 사용
                if self._is_near_pose(transformed_pose, self._last_stage2_goal, pos_tol, yaw_tol):
                    # Stage-3 활성화 여부 확인
                    if bool(self.get_parameter('stage3_enabled').value):
                        self._stage = 3
                        self.get_logger().info('Stage-2 complete -> Stage-3')
                        # Stage-3에서는 전진 모드 (최종 정렬)
                        self._publish_reverse_flag(False)
                        return True
                    else:
                        self.get_logger().info('Stage-2 complete -> 주차 완료 (Stage-3 비활성화)')
                        # Stage-3가 비활성화된 경우 주차 완료로 처리
                        return False

        elif self._stage == 3:
            # 목표: 슬롯 중심에서 yaw 정렬
            g3 = self._last_stage3_goal or self._compute_stage3_goal(self._parking_pose)
            if g3 is not None:
                # 변환된 pose 사용
                if self._is_near_pose(transformed_pose, g3, pos_tol, yaw_tol):
                    # 최종 완료. 여기서는 유지(또는 1로 리셋)
                    self.get_logger().info('Stage-3 complete (mission done)')
                    return False

        return False

    def _is_near_pose(self, a: PoseStamped, b: PoseStamped, pos_tol: float, yaw_tol: float) -> bool:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dist = math.hypot(dx, dy)
        ya = quaternion_to_yaw(a.pose.orientation)
        yb = quaternion_to_yaw(b.pose.orientation)
        dyaw = abs(wrap_to_pi(ya - yb))
        return (dist <= pos_tol) and (dyaw <= yaw_tol)

    # ───────────────────────────── Utilities ─────────────────────────────
    # _min_distance_to_walls 함수 제거 - _min_distance_to_cones로 대체됨

    def _compute_stage1_path(self, current_pose: Odometry, goal: PoseStamped) -> Optional[Path]:
        """현재 위치에서 Stage-1 목표까지 직선 경로 생성
        Args:
            current_pose: 현재 차량 위치 (오도메트리)
            goal: Stage-1 목표 위치
        Returns:
            생성된 경로 또는 None (실패 시)
        """
        if current_pose is None:
            self.get_logger().warn("현재 위치 정보 없음, Stage-1 경로 생성 중단")
            return None
        
        # 오도메트리를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(current_pose)
        if transformed_pose is None:
            self.get_logger().warn("오도메트리 map 프레임 변환 실패")
            return None
        
        self.get_logger().info(f"Stage-1 경로 - 현재 위치 (map): ({transformed_pose.pose.position.x:.2f}, {transformed_pose.pose.position.y:.2f})")
        self.get_logger().info(f"Stage-1 경로 - 목표 위치: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        # 경로 메시지 생성
        path = Path()
        path.header = goal.header
        
        # 현재 위치와 목표 위치 (모두 map 프레임)
        x0 = transformed_pose.pose.position.x
        y0 = transformed_pose.pose.position.y
        x1 = goal.pose.position.x
        y1 = goal.pose.position.y
        
        # 직선 경로 생성
        dx = x1 - x0
        dy = y1 - y0
        distance = math.hypot(dx, dy)
        
        if distance < 1e-6:  # 거리가 너무 작으면
            self.get_logger().warn("Stage-1 경로 거리가 너무 작음")
            return None
        
        # 경로 해상도에 따른 점 생성
        ds = float(self.get_parameter('path_resolution').value)
        n_points = max(1, int(distance / ds))
        
        for i in range(n_points + 1):
            t = i / n_points
            px = x0 + t * dx
            py = y0 + t * dy
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(math.atan2(dy, dx))
            path.poses.append(pose)
        
        # 충돌 검사
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        vehicle_length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(vehicle_width, vehicle_length) + safety_margin
        
        collided = False
        for p in path.poses:
            d = self._min_distance_to_cones(p.pose.position.x, p.pose.position.y)
            if d < eff_radius:
                collided = True
                break
        
        if collided:
            self.get_logger().warn('Stage-1 경로가 콘과 충돌할 수 있음. front_margin/clear_lateral 증가 고려.')
        
        self.get_logger().info(f"Stage-1 경로 생성 완료: {len(path.poses)}개 점")
        return path

    # ───────────────── Stage-2 Guided reverse (no fixed goal) ────────────
    def _to_slot_frame(self, px: float, py: float, center: Tuple[float, float], yaw_slot: float) -> Tuple[float, float]:
        dx = px - center[0]
        dy = py - center[1]
        c = math.cos(yaw_slot)
        s = math.sin(yaw_slot)
        x_l =  c * dx + s * dy
        y_l = -s * dx + c * dy
        return x_l, y_l

    def _inside_slot(self, px: float, py: float, center: Tuple[float, float], yaw_slot: float, L: float, W: float, margin: float) -> bool:
        x_l, y_l = self._to_slot_frame(px, py, center, yaw_slot)
        return (-L/2.0 + margin <= x_l <= L/2.0 - margin) and (-W/2.0 + margin <= y_l <= W/2.0 - margin)

    def _compute_stage2_path_guided(self, start_pose: Odometry, open_slot_pose: Optional[PoseStamped]) -> Optional[Path]:
        self.get_logger().info("🔄 === Stage-2 가이드 경로 생성 시작 ===")
        
        if start_pose is None or open_slot_pose is None:
            self.get_logger().error("❌ 입력 검증 실패: start_pose 또는 open_slot_pose가 None")
            return None
            
        self.get_logger().info(f"📥 입력 데이터 - start_pose: ({start_pose.pose.pose.position.x:.2f}, {start_pose.pose.pose.position.y:.2f})")
        self.get_logger().info(f"📥 입력 데이터 - open_slot_pose: ({open_slot_pose.pose.position.x:.2f}, {open_slot_pose.pose.position.y:.2f})")
        
        # Slot geometry
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        center = (open_slot_pose.pose.position.x, open_slot_pose.pose.position.y)
        if self._parking_area_idx is not None:
            L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
        else:
            L, W = 5.0, 2.5  # 기본값

        self.get_logger().info(f"📍 주차 공간 정보 - 중심: ({center[0]:.2f}, {center[1]:.2f}), 방향: {yaw_slot:.2f}, 크기: {L:.2f}x{W:.2f}")

        # Params
        ds = float(self.get_parameter('path_resolution').value)
        R = float(self.get_parameter('turn_radius').value)
        k_lat = float(self.get_parameter('stage2_k_lat').value)
        k_yaw = float(self.get_parameter('stage2_k_yaw').value)
        margin = float(self.get_parameter('stage2_inside_margin').value)
        max_steps = int(self.get_parameter('stage2_max_steps').value)
        Ld = float(self.get_parameter('stage2_lookahead').value)
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        safety_margin = float(self.get_parameter('safety_margin').value)

        self.get_logger().info(f"🔧 가이드 경로 파라미터 - 해상도={ds:.3f}, 회전반경={R:.2f}, 안전여유={safety_margin:.2f}")

        # Initialize - UTM 좌표를 map 프레임으로 변환
        self.get_logger().info("🔄 UTM→Map TF 변환 중...")
        transformed_pose = self._transform_odom_to_map(start_pose)
        if transformed_pose is None:
            self.get_logger().error("❌ TF 변환 실패")
            return None
            
        x = transformed_pose.pose.position.x
        y = transformed_pose.pose.position.y
        yaw = quaternion_to_yaw(transformed_pose.pose.orientation)
        path = Path(); path.header = open_slot_pose.header

        self.get_logger().info(f"✅ TF 변환 성공 - 현재 위치 (map): ({x:.2f}, {y:.2f}), 방향: {yaw:.2f}")

        # Stage 2 골을 먼저 계산해서 target으로 사용
        self.get_logger().info("🎯 Stage-2 목표 계산 중...")
        stage2_goal = self._compute_stage2_goal(open_slot_pose)
        if stage2_goal is None:
            self.get_logger().error("❌ Stage-2 목표 계산 실패")
            return None
        
        # Stage 2 골의 좌표를 target으로 사용
        target_world = (stage2_goal.pose.position.x, stage2_goal.pose.position.y)
        
        self.get_logger().info(f"✅ Stage-2 목표 계산 성공 - 목표: ({target_world[0]:.2f}, {target_world[1]:.2f})")
        self.get_logger().info(f"📏 목표까지 거리: {math.hypot(target_world[0] - x, target_world[1] - y):.2f}m")

        # 직선 경로 생성 (Stage 1과 동일한 방식)
        x0, y0 = x, y
        dx = target_world[0] - x0
        dy = target_world[1] - y0
        distance = math.hypot(dx, dy)
        n_points = max(1, int(distance / ds))
        
        self.get_logger().info(f"🔄 직선 경로 생성 - 거리: {distance:.2f}m, 점 개수: {n_points}")
        
        collision_detected = False
        collision_point = None
        for i in range(n_points + 1):
            t = i / n_points
            px = x0 + t * dx
            py = y0 + t * dy
            
            # 충돌 검사
            width = float(self.get_parameter('vehicle_width').value)
            length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
            eff_radius = 0.5 * math.hypot(width, length) + safety_margin
            cone_distance = self._min_distance_to_cones(px, py)
            
            if cone_distance < eff_radius:
                self.get_logger().error(f"❌ 충돌 감지 - 점 {i}: ({px:.2f}, {py:.2f}), 콘까지 거리: {cone_distance:.2f}m < {eff_radius:.2f}m")
                collision_detected = True
                collision_point = i
                break
                
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(math.atan2(dy, dx))
            path.poses.append(pose)

        if collision_detected:
            self.get_logger().error(f"❌ 가이드 경로 충돌 감지됨 (충돌점: {collision_point})")
        else:
            self.get_logger().info("✅ 가이드 경로 충돌 검사 통과")

        self.get_logger().info(f"✅ Stage-2 가이드 경로 생성 완료: {len(path.poses)}개 점")
        self.get_logger().info("🔄 === Stage-2 가이드 경로 생성 종료 ===")
        return path if path.poses else None

    # ───────────────────────────── Stage-3 (align heading) ───────────────
    def _compute_stage3_path(self, start_pose: Odometry, yaw_slot: float) -> Optional[Path]:
        # UTM 좌표를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(start_pose)
        if transformed_pose is None:
            self.get_logger().warn("DEBUG: Failed to transform odom to map frame in stage3")
            return None
            
        yaw_s = quaternion_to_yaw(transformed_pose.pose.orientation)
        dyaw = wrap_to_pi(yaw_slot - yaw_s)
        tol = math.radians(float(self.get_parameter('stage3_yaw_tol_deg').value)) if self.has_parameter('stage3_yaw_tol_deg') else math.radians(5.0)
        ds = float(self.get_parameter('path_resolution').value)
        R = float(self.get_parameter('stage3_turn_radius').value) if self.has_parameter('stage3_turn_radius') else 2.0
        forward_extra = float(self.get_parameter('stage3_forward').value) if self.has_parameter('stage3_forward') else 0.4

        path = Path(); path.header = transformed_pose.header

        # Slot geometry and safety for in-slot and collision guard
        center = None
        L = None
        W = None
        if self._parking_pose is not None:
            center = (self._parking_pose.pose.position.x, self._parking_pose.pose.position.y)
            L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx) if self._parking_area_idx is not None else (5.0, 2.5)
        margin = float(self.get_parameter('stage2_inside_margin').value) if self.has_parameter('stage2_inside_margin') else 0.2
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin

        if abs(dyaw) <= tol:
            x0 = transformed_pose.pose.position.x
            y0 = transformed_pose.pose.position.y
            n_steps = max(1, int(forward_extra / max(ds, 1e-3)))
            self.get_logger().info(f"Stage-3: Generating {n_steps} steps from ({x0:.2f}, {y0:.2f}) with forward_extra={forward_extra:.2f}")
            
            # 슬롯 정보 디버그
            if center is not None and L is not None and W is not None:
                self.get_logger().info(f"Stage-3: Slot center=({center[0]:.2f}, {center[1]:.2f}), L={L:.2f}, W={W:.2f}, margin={margin:.2f}")
                self.get_logger().info(f"Stage-3: Slot bounds: x=[{center[0]-L/2+margin:.2f}, {center[0]+L/2-margin:.2f}], y=[{center[1]-W/2+margin:.2f}, {center[1]+W/2-margin:.2f}]")
            
            for i in range(n_steps + 1):
                s = (forward_extra * i) / max(1, n_steps)
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position.x = x0 + s * math.cos(yaw_s)
                pose.pose.position.y = y0 + s * math.sin(yaw_s)
                pose.pose.orientation = yaw_to_quaternion(yaw_s)
                
                # Guards: stay inside slot and avoid cones
                if center is not None and L is not None and W is not None:
                    inside_slot = self._inside_slot(pose.pose.position.x, pose.pose.position.y, center, yaw_slot, L, W, margin)
                    if not inside_slot:
                        self.get_logger().warn(f'Stage-3: Point ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) outside slot bounds. Breaking.')
                        break
                
                cone_dist = self._min_distance_to_cones(pose.pose.position.x, pose.pose.position.y)
                if cone_dist < eff_radius:
                    self.get_logger().warn(f'Stage-3: Point ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) too close to cone (dist={cone_dist:.2f} < {eff_radius:.2f}). Breaking.')
                    break
                
                path.poses.append(pose)
            
            # 최소한 하나의 점은 생성
            if not path.poses:
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position.x = x0
                pose.pose.position.y = y0
                pose.pose.orientation = yaw_to_quaternion(yaw_s)
                path.poses.append(pose)
                self.get_logger().warn('Stage-3: Created minimal path with start pose only')
            
            self.get_logger().info(f"Stage-3: Generated {len(path.poses)} points")
            return path

        # 직선 경로 생성 (Stage 1, 2와 동일한 방식)
        x0 = transformed_pose.pose.position.x
        y0 = transformed_pose.pose.position.y
        
        # 목표: 현재 위치에서 yaw_slot 방향으로 forward_extra만큼 직진
        target_x = x0 + forward_extra * math.cos(yaw_slot)
        target_y = y0 + forward_extra * math.sin(yaw_slot)
        
        dx = target_x - x0
        dy = target_y - y0
        distance = math.hypot(dx, dy)
        n_points = max(1, int(distance / ds))
        
        self.get_logger().info(f"DEBUG: Stage3 straight line - distance: {distance:.2f}, points: {n_points}")
        
        for i in range(n_points + 1):
            t = i / n_points
            px = x0 + t * dx
            py = y0 + t * dy
            
            # 충돌 검사
            if center is not None and L is not None and W is not None:
                if not self._inside_slot(px, py, center, yaw_slot, L, W, margin):
                    self.get_logger().warn(f'Stage-3 straight line would exit slot bounds at point {i}. Stopping early.')
                    break
                    
            if self._min_distance_to_cones(px, py) < eff_radius:
                self.get_logger().warn(f'Stage-3 straight line would collide with cone at point {i}. Stopping early.')
                break
                
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw_slot)
            path.poses.append(pose)

        return path

    # ───────────────────────────── Stage-2 (reverse) ─────────────────────
    def _compute_stage2_goal(self, parking_pose: Optional[PoseStamped]) -> Optional[PoseStamped]:
        self.get_logger().info("🎯 Stage-2 목표 계산 시작")
        
        if parking_pose is None:
            self.get_logger().error("❌ parking_pose가 None입니다")
            return None
            
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        center = (parking_pose.pose.position.x, parking_pose.pose.position.y)
        self.get_logger().info(f"📍 주차 공간 중심: ({center[0]:.2f}, {center[1]:.2f}), 방향: {yaw_slot:.2f}")
        
        goal = PoseStamped()
        goal.header = parking_pose.header
        use_map = bool(self.get_parameter('stage2_use_map_y_offset').value)
        self.get_logger().info(f"🔧 Stage-2 목표 모드: {'map_y_offset' if use_map else 'back_offset'}")
        
        if use_map:
            offset_y = float(self.get_parameter('stage2_goal_offset_y').value)
            goal.pose.position.x = center[0]
            goal.pose.position.y = center[1] + offset_y
            self.get_logger().info(f"📍 Map Y 오프셋 모드: offset_y={offset_y:.2f}")
        else:
            back = float(self.get_parameter('stage2_back_offset').value)
            u_long = (math.cos(yaw_slot), math.sin(yaw_slot))
            goal.pose.position.x = center[0] - back * u_long[0]
            goal.pose.position.y = center[1] - back * u_long[1]
            self.get_logger().info(f"📍 Back 오프셋 모드: back={back:.2f}")
            
        goal.pose.position.z = 0.0
        self.get_logger().info(f"📍 계산된 목표 위치: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        # Stage-2 goal heading: match Stage-1 heading if available; otherwise fallback to slot yaw
        if self._last_stage1_goal is not None:
            goal.pose.orientation = self._last_stage1_goal.pose.orientation
            self.get_logger().info("🧭 Stage-1 목표 방향 사용")
        else:
            goal.pose.orientation = yaw_to_quaternion(yaw_slot)
            self.get_logger().info("🧭 주차 공간 방향 사용")
        
        # 충돌 검사: Stage 2 골 위치에 충돌이 있으면 None 반환
        vehicle_radius = float(self.get_parameter('vehicle_width').value) / 2.0 + float(self.get_parameter('safety_margin').value)
        cone_distance = self._min_distance_to_cones(goal.pose.position.x, goal.pose.position.y)
        self.get_logger().info(f"🚗 차량 반경: {vehicle_radius:.2f}m, 콘까지 거리: {cone_distance:.2f}m")
        
        if cone_distance < vehicle_radius:
            self.get_logger().error(f"❌ Stage-2 목표 위치 충돌 감지: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
            return None
        
        self.get_logger().info("✅ Stage-2 목표 계산 성공")
        return goal

    def _compute_stage3_goal(self, parking_pose: Optional[PoseStamped]) -> Optional[PoseStamped]:
        if parking_pose is None:
            return None
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        goal = PoseStamped()
        goal.header = parking_pose.header
        goal.pose.position.x = parking_pose.pose.position.x
        goal.pose.position.y = parking_pose.pose.position.y
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw_slot)
        return goal

    def _compute_stage3_path_from_stage2(self, start_pose: Odometry, parking_pose: PoseStamped) -> Optional[Path]:
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        center = (parking_pose.pose.position.x, parking_pose.pose.position.y)
        ds = float(self.get_parameter('path_resolution').value)
        R = float(self.get_parameter('stage3_turn_radius').value) if self.has_parameter('stage3_turn_radius') else 2.0
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin
        if self._parking_area_idx is not None:
            L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
        else:
            L, W = 5.0, 2.5
        margin = float(self.get_parameter('stage2_inside_margin').value) if self.has_parameter('stage2_inside_margin') else 0.2

        # 1) Align yaw from start to yaw_slot via arc
        yaw_s = quaternion_to_yaw(start_pose.pose.pose.orientation)
        dyaw = wrap_to_pi(yaw_slot - yaw_s)
        path = Path(); path.header = parking_pose.header
        turn_left = dyaw > 0.0
        n_left_s = (-math.sin(yaw_s), math.cos(yaw_s))
        Sx = start_pose.pose.pose.position.x
        Sy = start_pose.pose.pose.position.y
        if turn_left:
            Cx = Sx + R * n_left_s[0]
            Cy = Sy + R * n_left_s[1]
        else:
            Cx = Sx - R * n_left_s[0]
            Cy = Sy - R * n_left_s[1]
        n_steps = max(1, int((abs(dyaw) * R) / max(ds, 1e-3)))
        for j in range(n_steps + 1):
            th = yaw_s + dyaw * (j / max(1, n_steps))
            nL = (-math.sin(th), math.cos(th))
            if turn_left:
                px = Cx - R * nL[0]
                py = Cy - R * nL[1]
            else:
                px = Cx + R * nL[0]
                py = Cy + R * nL[1]
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation = yaw_to_quaternion(th)
            if not self._inside_slot(px, py, center, yaw_slot, L, W, margin):
                break
            if self._min_distance_to_cones(px, py) < eff_radius:
                break
            path.poses.append(pose)

        # 2) Straight from last arc pose to slot center along yaw_slot
        if not path.poses:
            # no arc added; use start pose as last
            last_x = Sx
            last_y = Sy
        else:
            last_x = path.poses[-1].pose.position.x
            last_y = path.poses[-1].pose.position.y
        dx = center[0] - last_x
        dy = center[1] - last_y
        L_line = math.hypot(dx, dy)
        n_line = max(1, int(L_line / max(ds, 1e-3)))
        for i in range(1, n_line + 1):
            t = i / max(1, n_line)
            px = last_x + t * dx
            py = last_y + t * dy
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation = yaw_to_quaternion(yaw_slot)
            if not self._inside_slot(px, py, center, yaw_slot, L, W, margin):
                break
            if self._min_distance_to_cones(px, py) < eff_radius:
                break
            path.poses.append(pose)

        return path if path.poses else None

    def _compute_stage2_path(self, start_pose: Odometry, goal: PoseStamped) -> Optional[Path]:
        if start_pose is None:
            return None
        # Parameters
        R = float(self.get_parameter('turn_radius').value)
        ds = float(self.get_parameter('path_resolution').value)
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        safety_margin = float(self.get_parameter('safety_margin').value)

        yaw_s = quaternion_to_yaw(start_pose.pose.pose.orientation)
        yaw_g = quaternion_to_yaw(goal.pose.orientation)
        # 원하는 것은 시작(yaw_s = yaw_slot+30°)에서 목표(yaw_g=yaw_slot)로 30° 만큼 우회전하며 후진
        yaw_delta = wrap_to_pi(yaw_g - yaw_s)
        # 우회전 필요(음수). 양수면 좌회전으로 처리
        turn_right = yaw_delta < 0.0
        alpha = abs(yaw_delta)

        # 회전 중심: 시작점에서 법선 방향으로 R만큼 (우측: -n_left, 좌측: +n_left)
        n_left_s = (-math.sin(yaw_s), math.cos(yaw_s))
        Sx = start_pose.pose.pose.position.x
        Sy = start_pose.pose.pose.position.y
        if turn_right:
            Cx = Sx - R * n_left_s[0]
            Cy = Sy - R * n_left_s[1]
        else:
            Cx = Sx + R * n_left_s[0]
            Cy = Sy + R * n_left_s[1]

        # 경로 메시지
        path = Path()
        path.header = goal.header


        # 1) 후진 원호: yaw_s -> yaw_g
        n_steps = max(1, int((alpha * R) / ds))
        for j in range(n_steps + 1):
            th = yaw_s + (yaw_delta) * (j / max(1, n_steps))
            nL = (-math.sin(th), math.cos(th))
            # 원 위의 점: 우회전이면 P = C + R*n_left(th), 좌회전이면 P = C - R*n_left(th)
            if turn_right:
                px = Cx + R * nL[0]
                py = Cy + R * nL[1]
            else:
                px = Cx - R * nL[0]
                py = Cy - R * nL[1]
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation = yaw_to_quaternion(th)
            path.poses.append(pose)

        # 2) 목표까지 직선(계속 후진)
        x1 = path.poses[-1].pose.position.x
        y1 = path.poses[-1].pose.position.y
        x2 = goal.pose.position.x
        y2 = goal.pose.position.y
        dx = x2 - x1
        dy = y2 - y1
        L_line = math.hypot(dx, dy)
        n_line = max(1, int(L_line / ds))
        for i in range(1, n_line + 1):
            t = i / max(1, n_line)
            px = x1 + t * dx
            py = y1 + t * dy
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation = yaw_to_quaternion(yaw_g)
            path.poses.append(pose)

        # 충돌 체크
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin
        for p in path.poses:
            if self._min_distance_to_cones(p.pose.position.x, p.pose.position.y) < eff_radius:
                self.get_logger().warn('Stage-2 path may collide with cones. Tune clearances/turn_radius.')
                break

        return path

    def _compute_stage2_path_s_curve(self, current_odom: Odometry, parking_pose: PoseStamped) -> Optional[Path]:
        """S자 형태의 Stage-2 경로 생성 (직선 경로 로직 참고하여 단순화)
        Args:
            current_odom: 현재 차량 위치 (오도메트리)
            parking_pose: 주차 공간 위치
        Returns:
            생성된 S자 경로 또는 None (실패 시)
        """
        self.get_logger().info("🔄 === S자 경로 생성 시작 ===")
        
        # 직선 경로와 동일한 입력 검증
        if current_odom is None or parking_pose is None:
            self.get_logger().error("❌ 입력 검증 실패: current_odom 또는 parking_pose가 None")
            return None
        
        self.get_logger().info(f"📥 입력 데이터 - odom: ({current_odom.pose.pose.position.x:.2f}, {current_odom.pose.pose.position.y:.2f})")
        self.get_logger().info(f"📥 입력 데이터 - parking: ({parking_pose.pose.position.x:.2f}, {parking_pose.pose.position.y:.2f})")
        
        try:
            # 1. TF 변환 (직선 경로와 동일한 로직)
            self.get_logger().info("🔄 UTM→Map TF 변환 중...")
            transformed_pose = self._transform_odom_to_map(current_odom)
            if transformed_pose is None:
                self.get_logger().error("❌ TF 변환 실패")
                return None
            
            # 2. Stage-1 목표를 시작점으로 사용 (차량 현재 위치가 아닌)
            if self._last_stage1_goal is None:
                self.get_logger().error("❌ Stage-1 목표가 없음 - Stage-1을 먼저 완료해야 함")
                return None
            
            start_x = self._last_stage1_goal.pose.position.x
            start_y = self._last_stage1_goal.pose.position.y
            
            self.get_logger().info(f"✅ Stage-1 목표를 시작점으로 사용: ({start_x:.2f}, {start_y:.2f})")
            
            # 3. 주차 공간을 끝점으로 사용 (Stage-2 목표가 아닌)
            goal_x = parking_pose.pose.position.x
            goal_y = parking_pose.pose.position.y
            
            self.get_logger().info(f"✅ 주차 공간을 끝점으로 사용: ({goal_x:.2f}, {goal_y:.2f})")
            
            # 4. S자 경로 파라미터
            resolution = float(self.get_parameter('s_curve_resolution').value)
            
            self.get_logger().info(f"🔧 S자 경로 파라미터 - 해상도={resolution:.2f}")
            
            # 5. Path 메시지 생성 (직선 경로와 동일한 헤더 설정)
            path = Path()
            path.header = parking_pose.header  # 직선 경로와 동일
            
            # 6. 베지어 곡선으로 S자 경로 생성 (방향 정보 완전 제거)
            self.get_logger().info("🔄 Stage-1 목표 → 주차공간 베지어 곡선 경로 생성 중...")
            self.get_logger().info(f"📍 Stage-1 목표 (시작점): ({start_x:.2f}, {start_y:.2f})")
            self.get_logger().info(f"📍 주차공간 (끝점): ({goal_x:.2f}, {goal_y:.2f})")
            
            combined_points = self._generate_simple_curve(
                start_x, start_y, 0.0,  # 시작점: Stage-1 목표, 방향 무시
                goal_x, goal_y, 0.0,    # 끝점: 주차공간 포즈, 방향 무시
                resolution
            )
            
            if combined_points is None:
                self.get_logger().error("❌ 베지어 곡선 경로 생성 실패")
                return None
            
            self.get_logger().info(f"✅ 베지어 곡선 경로 생성 완료: {len(combined_points)}개 점")
            
            # 9. 충돌 검사 (직선 경로와 동일한 방식)
            self.get_logger().info("🚗 S자 경로 충돌 검사 중...")
            vehicle_width = float(self.get_parameter('vehicle_width').value)
            vehicle_length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
            safety_margin = float(self.get_parameter('safety_margin').value)
            eff_radius = 0.5 * math.hypot(vehicle_width, vehicle_length) + safety_margin
            
            self.get_logger().info(f"🚗 충돌 검사 파라미터 - 차량폭={vehicle_width:.2f}, 차량길이={vehicle_length:.2f}, 안전여유={safety_margin:.2f}, 유효반경={eff_radius:.2f}")
            
            collision_detected = False
            collision_point = None
            for i, point in enumerate(combined_points):
                cone_distance = self._min_distance_to_cones(point[0], point[1])
                if cone_distance < eff_radius:
                    self.get_logger().error(f"❌ 충돌 감지 - 점 {i}: ({point[0]:.2f}, {point[1]:.2f}), 콘까지 거리: {cone_distance:.2f}m < {eff_radius:.2f}m")
                    collision_detected = True
                    collision_point = i
                    break
            
            if collision_detected:
                self.get_logger().error(f"❌ S자 경로 충돌 감지됨 (충돌점: {collision_point})")
                return None
            
            self.get_logger().info("✅ S자 경로 충돌 검사 통과")
            
            # 10. Path 메시지에 점들 추가 (방향 정보 없이 순수 좌표만 사용)
            self.get_logger().info("📝 Path 메시지 생성 중...")
            for i, point in enumerate(combined_points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                
                # 방향 계산: 현재 점에서 다음 점으로의 방향 (마지막 점은 직선 방향 사용)
                if i < len(combined_points) - 1:
                    next_point = combined_points[i + 1]
                    dx = next_point[0] - point[0]
                    dy = next_point[1] - point[1]
                    heading = math.atan2(dy, dx)
                else:
                    # 마지막 점은 직선 방향 사용 (방향 정보 제거)
                    heading = math.atan2(goal_y - start_y, goal_x - start_x)
                
                pose.pose.orientation = yaw_to_quaternion(heading)
                path.poses.append(pose)
            
            self.get_logger().info(f"✅ S자 경로 생성 완료: {len(path.poses)}개 점")
            self.get_logger().info("🔄 === S자 경로 생성 종료 ===")
            return path
            
        except Exception as e:
            self.get_logger().error(f"❌ S자 경로 생성 예외 발생: {e}")
            import traceback
            self.get_logger().error(f"상세 오류: {traceback.format_exc()}")
            return None

    def _generate_simple_curve(self, start_x: float, start_y: float, start_heading: float,
                              end_x: float, end_y: float, end_heading: float,
                              resolution: float) -> Optional[List[Tuple[float, float]]]:
        """부드러운 곡선 경로 생성 (3차 함수 기반 베지어 곡선) - 순수 좌표만 반환
        Args:
            start_x, start_y: 시작점 좌표
            start_heading: 시작 방향 (제어점 계산용)
            end_x, end_y: 끝점 좌표  
            end_heading: 끝 방향 (제어점 계산용)
            resolution: 경로 해상도
        Returns:
            곡선 위의 점들 리스트 [(x, y), ...] 또는 None (방향 정보 제외)
        """
        try:
            # 시작점과 끝점 사이의 거리 계산
            dx = end_x - start_x
            dy = end_y - start_y
            distance = math.hypot(dx, dy)
            
            self.get_logger().info(f"🔄 3차 곡선 생성 시작 - 거리: {distance:.2f}m, 해상도: {resolution:.3f}m")
            
            if distance < 1e-6:
                self.get_logger().warn("곡선 생성: 시작점과 끝점이 너무 가까움")
                return None
            
            # 제어점 계산 (S자 형태를 위한 수직 오프셋)
            control_offset = distance * 0.3  # 거리의 30%만큼 오프셋 (더 뚜렷한 곡선)
            
            # 직선 방향과 수직 방향 단위 벡터 계산
            dx_unit = dx / distance if distance > 1e-6 else 0.0
            dy_unit = dy / distance if distance > 1e-6 else 0.0
            
            # 수직 방향 단위 벡터 (시계방향 90도 회전)
            perp_x = -dy_unit
            perp_y = dx_unit
            
            # 시작점에서 직선 방향 + 수직 방향으로 제어점 (S자 형태)
            control1_x = start_x + control_offset * dx_unit + control_offset * perp_x
            control1_y = start_y + control_offset * dy_unit + control_offset * perp_y
            
            # 끝점에서 직선 방향의 반대 + 수직 방향으로 제어점 (S자 형태)
            control2_x = end_x - control_offset * dx_unit + control_offset * perp_x
            control2_y = end_y - control_offset * dy_unit + control_offset * perp_y
            
            # 베지어 곡선 위의 점들 생성
            points = []
            n_points = max(10, int(distance / resolution))  # 최소 10개 점으로 해상도 향상
            
            self.get_logger().info(f"📍 제어점 계산 완료 - 오프셋: {control_offset:.2f}m, 점 개수: {n_points}")
            self.get_logger().info(f"📍 제어점1: ({control1_x:.2f}, {control1_y:.2f})")
            self.get_logger().info(f"📍 제어점2: ({control2_x:.2f}, {control2_y:.2f})")
            
            for i in range(n_points + 1):
                t = i / n_points
                
                # 베지어 곡선 공식 (3차) - 순수 좌표만 계산
                x = (1-t)**3 * start_x + 3*(1-t)**2*t * control1_x + 3*(1-t)*t**2 * control2_x + t**3 * end_x
                y = (1-t)**3 * start_y + 3*(1-t)**2*t * control1_y + 3*(1-t)*t**2 * control2_y + t**3 * end_y
                
                # 방향 정보 없이 순수 좌표만 저장
                points.append((x, y))
            
            self.get_logger().info(f"✅ 3차 곡선 생성 완료 - 총 {len(points)}개 점 생성")
            return points
            
        except Exception as e:
            self.get_logger().error(f"간단한 곡선 생성 실패: {e}")
            return None

    def _generate_s_curve_arc(self, start_x: float, start_y: float, start_heading: float,
                             end_x: float, end_y: float, end_heading: float,
                             radius: float, resolution: float) -> Optional[List[Tuple[float, float, float]]]:
        """S자 경로의 개별 원호 생성
        Args:
            start_x, start_y: 시작점 좌표
            start_heading: 시작 방향
            end_x, end_y: 끝점 좌표  
            end_heading: 끝 방향
            radius: 원호 반경
            resolution: 경로 해상도
        Returns:
            원호 위의 점들 리스트 [(x, y, heading), ...] 또는 None
        """
        try:
            # 원호 중심점 계산 (간단한 접근법)
            # 시작점에서 수직 방향으로 radius만큼 이동하여 중심점 계산
            center_x = start_x + radius * math.sin(start_heading)
            center_y = start_y - radius * math.cos(start_heading)
            
            # 시작각과 끝각 계산
            start_angle = start_heading - math.pi/2
            end_angle = end_heading - math.pi/2
            
            # 각도 정규화
            while end_angle - start_angle > math.pi:
                end_angle -= 2 * math.pi
            while end_angle - start_angle < -math.pi:
                end_angle += 2 * math.pi
            
            # 원호 위의 점들 생성
            points = []
            angle_step = resolution / radius  # 각도 스텝
            
            current_angle = start_angle
            while abs(current_angle - end_angle) > angle_step:
                x = center_x + radius * math.cos(current_angle)
                y = center_y + radius * math.sin(current_angle)
                heading = current_angle + math.pi/2
                points.append((x, y, heading))
                current_angle += angle_step if end_angle > start_angle else -angle_step
            
            # 마지막 점 추가
            x = center_x + radius * math.cos(end_angle)
            y = center_y + radius * math.sin(end_angle)
            points.append((x, y, end_heading))
            
            return points
            
        except Exception as e:
            self.get_logger().error(f"원호 생성 실패: {e}")
            return None

    def _check_s_curve_collision(self, points: List[Tuple[float, float, float]]) -> bool:
        """S자 경로의 충돌 검사
        Args:
            points: 검사할 경로 점들 [(x, y, heading), ...]
        Returns:
            충돌이 감지되면 True, 아니면 False
        """
        try:
            collision_resolution = float(self.get_parameter('s_curve_collision_resolution').value)
            vehicle_radius = float(self.get_parameter('s_curve_vehicle_radius').value)
            
            for point in points[::max(1, int(collision_resolution / 0.1))]:  # 해상도에 따라 샘플링
                if self._min_distance_to_cones(point[0], point[1]) < vehicle_radius:
                    return True
            return False
            
        except Exception as e:
            self.get_logger().error(f"S자 경로 충돌 검사 실패: {e}")
            return True  # 에러 시 안전을 위해 충돌로 판정


    def _stage2_path_shape_wallS(self, current_odom: Odometry, parking_pose: PoseStamped) -> Optional[Path]:
        """
        '벽 따라 내려오다 S로 슬롯 진입' 경로.
        - 시작 포즈 기준으로 벽(로컬 x=0)을 왼/오 어느쪽이든 자동 판단.
        - s in [0,D]로 곡률 κ(s) 적분 → (x,y), 초반 s<=s_hold 구간엔 벽 클리어런스(x>=d_cl) 부드럽게 유지.
        """
        self.get_logger().info("🔄 === 벽 따라 S-curve 경로 생성 시작 ===")
        
        # 0) 입력/시작·목표
        start_map = self._transform_odom_to_map(current_odom)
        if start_map is None: 
            self.get_logger().error("❌ TF 변환 실패")
            return None
        xs = float(start_map.pose.position.x)
        ys = float(start_map.pose.position.y)
        psis = quaternion_to_yaw(start_map.pose.orientation)
        
        self.get_logger().info(f"📍 시작 위치: ({xs:.2f}, {ys:.2f}), 방향: {psis:.2f}")

        g2 = self._compute_stage2_goal(parking_pose)
        if g2 is None: 
            self.get_logger().error("❌ Stage-2 목표 계산 실패")
            return None
        yaw_goal = quaternion_to_yaw(g2.pose.orientation)
        
        self.get_logger().info(f"📍 목표 위치: ({g2.pose.position.x:.2f}, {g2.pose.position.y:.2f}), 방향: {yaw_goal:.2f}")

        # 1) 파라미터
        mode = str(self.get_parameter('stage2_shape_mode').value)
        D = float(self.get_parameter('shape_total_length').value)
        ds = float(self.get_parameter('shape_ds').value)
        k_req = float(self.get_parameter('shape_kappa_max').value)
        p_sw = float(self.get_parameter('shape_switch_pos').value)
        gamma = float(self.get_parameter('shape_gamma').value)

        d_cl = float(self.get_parameter('shape_wall_clearance').value)
        s_hold = float(self.get_parameter('shape_wall_hold_s').value)
        beta = float(self.get_parameter('shape_wall_softness').value)

        Lwb = float(self.get_parameter('wheelbase').value)
        dmax = math.radians(float(self.get_parameter('delta_max_deg').value))
        k_max_vehicle = math.tan(dmax) / max(Lwb, 1e-6)
        k_amp = min(k_req, 0.98 * k_max_vehicle)
        
        # 파라미터 로그 출력
        self.get_logger().info(f"🔧 벽 따라 S-curve 파라미터:")
        self.get_logger().info(f"   - 모드: {mode}")
        self.get_logger().info(f"   - 전체 길이: {D:.2f}m")
        self.get_logger().info(f"   - 곡률 최대: {k_req:.3f} (차량한계: {k_max_vehicle:.3f}, 사용: {k_amp:.3f})")
        self.get_logger().info(f"   - S전환 위치: {p_sw:.2f}")
        self.get_logger().info(f"   - 벽 클리어런스: {d_cl:.2f}m")
        self.get_logger().info(f"   - 벽 따라 거리: {s_hold:.2f}m")
        self.get_logger().info(f"   - 휠베이스: {Lwb:.2f}m, 최대조향각: {math.degrees(dmax):.1f}°")

        # 2) 로컬 프레임(시작 헤딩 0)로 적분 준비
        def raised_cosine(s, s0, w):
            z = abs(s - s0)
            if z > w: 
                return 0.0
            return 0.5 * (1.0 + math.cos(math.pi * (s - s0) / max(w, 1e-9)))

        def kappa_s_curve(s, sign):
            s1 = D * (0.3 * p_sw)
            w1 = D * (0.15 / max(gamma, 1e-6))
            s2 = D * (0.7 * p_sw + .3)
            w2 = D * (0.15 / max(gamma, 1e-6))
            return sign * k_amp * (raised_cosine(s, s1, w1) - raised_cosine(s, s2, w2))

        def kappa_single(s, sign):
            s0 = D * 0.5
            w = D * (0.25 / max(gamma, 1e-6))
            return sign * k_amp * raised_cosine(s, s0, w)

        # 목표 헤딩 쪽이 좌/우인지로 전체 부호 결정
        dpsi = wrap_to_pi(yaw_goal - psis)
        turn_sign = 1.0 if dpsi >= 0.0 else -1.0
        kappa_fn = (lambda s: kappa_s_curve(s, turn_sign)) if mode == 's_curve' else (lambda s: kappa_single(s, turn_sign))

        # 3) 적분 (s 진행). 초반엔 '벽 클리어런스'를 소프트하게 유지.
        N = max(2, int(D / max(ds, 1e-3)))
        x_l = 0.0
        y_l = 0.0
        psi = 0.0
        xs_l = [x_l]
        ys_l = [y_l]
        
        for i in range(1, N+1):
            s = i * ds
            psi += kappa_fn((i-0.5)*ds) * ds
            # 기본 적분 스텝
            x_next = x_l + math.cos(psi) * ds
            y_next = y_l + math.sin(psi) * ds

            # --- 벽 따라 내려오기 보정 ---
            # 로컬에서 '벽'을 x=0 라인으로 가정. (차가 벽의 +x 쪽에 있다고 생각)
            # s<=s_hold 동안 x >= d_cl를 부드럽게 강제: x <- softmax(x, d_cl).
            if s <= s_hold:
                # softmax(a,b) ~ (1/beta) log( exp(beta*a) + exp(beta*b) )
                x_safe = (1.0/beta) * math.log(math.exp(beta * x_next) + math.exp(beta * d_cl))
                # 부드러운 블렌딩(초반 강, s_hold로 갈수록 0)
                lam = 0.5 * (1.0 + math.cos(min(1.0, s/s_hold) * math.pi))  # s=0→1:1→0
                x_next = lam * x_safe + (1.0 - lam) * x_next

            x_l, y_l = x_next, y_next
            xs_l.append(x_l)
            ys_l.append(y_l)

        # 마지막 구간, 목표 헤딩으로 소폭 블렌딩(1~2도 보정용)
        alpha = 0.2
        psi_bias = alpha * wrap_to_pi(dpsi - psi)
        # 전체 궤적을 회전 보정하되 시작은 0 유지되도록 s비례 가중
        psis_l = [0.0]
        for i in range(1, len(xs_l)):
            t = i / max(1, len(xs_l)-1)
            psis_l.append((t * psi_bias))

        # 4) 로컬→월드
        c = math.cos(psis)
        s = math.sin(psis)
        path = Path()
        path.header = parking_pose.header
        
        for i in range(len(xs_l)):
            xl, yl = xs_l[i], ys_l[i]
            xw = xs + c*xl - s*yl
            yw = ys + s*xl + c*yl
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = xw
            pose.pose.position.y = yw
            pose.pose.position.z = 0.0
            
            if i < len(xs_l) - 1:
                xl2, yl2 = xs_l[i+1], ys_l[i+1]
                xw2 = xs + c*xl2 - s*yl2
                yw2 = ys + s*xl2 + c*yl2
                yaww = math.atan2(yw2 - yw, xw2 - xw)
            else:
                yaww = yaw_goal
            yaww = wrap_to_pi(yaww + psis_l[i])  # 헤딩 보정
            pose.pose.orientation = yaw_to_quaternion(yaww)
            path.poses.append(pose)

        # 5) 간단 충돌 검사(콘)
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin
        
        for p in path.poses:
            if self._min_distance_to_cones(p.pose.position.x, p.pose.position.y) < eff_radius:
                self.get_logger().warn("shape-wallS: 콘과 근접. total_length/kappa/clearance 조정 필요")
                break

        return path if path.poses else None

    def _compute_stage2_path_hardcoded(self, current_odom: Odometry, parking_pose: PoseStamped) -> Optional[Path]:
        """
        '벽을 따라 곧게 내려오다가 → S로 파고드는' 하드코딩 경로.
        - 로컬(슬롯 프레임)에서 y = wall_clear 라인을 벽이라 가정하고,
          straight_len만큼 -x로 이동 후, Cubic Bezier 한 번으로 슬롯 안쪽으로 진입.
        """
        self.get_logger().info("🔄 === 하드코딩 S-curve 경로 생성 시작 ===")
        
        # 입력 필수 확인 + TF
        if current_odom is None or parking_pose is None:
            self.get_logger().error("❌ 입력 검증 실패")
            return None
            
        start_map = self._transform_odom_to_map(current_odom)
        if start_map is None:
            self.get_logger().error("❌ TF 변환 실패")
            return None

        # 파라미터
        wall_clear   = float(self.get_parameter('hc_wall_clear').value)
        straight_len = float(self.get_parameter('hc_straight_len').value)
        c1_len       = float(self.get_parameter('hc_c1').value)
        c2_len       = float(self.get_parameter('hc_c2').value)
        goal_back    = float(self.get_parameter('hc_goal_back').value)
        inside_m     = float(self.get_parameter('hc_inside_margin').value)
        ds           = float(self.get_parameter('hc_ds').value)
        path_res     = float(self.get_parameter('path_resolution').value)

        self.get_logger().info(f"🔧 하드코딩 파라미터:")
        self.get_logger().info(f"   - 벽 클리어런스: {wall_clear:.2f}m")
        self.get_logger().info(f"   - 직선 길이: {straight_len:.2f}m")
        self.get_logger().info(f"   - 제어점1: {c1_len:.2f}m, 제어점2: {c2_len:.2f}m")
        self.get_logger().info(f"   - 목표 뒤로: {goal_back:.2f}m, 안쪽 여유: {inside_m:.2f}m")

        # 슬롯 프레임(원점=슬롯 중심, x=슬롯 진행방향, y=도로쪽(+))으로 변환
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        cx, cy   = parking_pose.pose.position.x, parking_pose.pose.position.y

        def to_slot(px, py):
            dx, dy = px - cx, py - cy
            c, s = math.cos(yaw_slot), math.sin(yaw_slot)
            return  c*dx + s*dy, -s*dx + c*dy
            
        def to_map(xl, yl):
            c, s = math.cos(yaw_slot), math.sin(yaw_slot)
            return cx + c*xl - s*yl, cy + s*xl + c*yl

        # Stage-1 목표를 시작점으로 사용 (일관성 유지)
        if self._last_stage1_goal is None:
            self.get_logger().error("❌ Stage-1 목표가 없음 - Stage-1을 먼저 완료해야 함")
            return None
            
        sx, sy = self._last_stage1_goal.pose.position.x, self._last_stage1_goal.pose.position.y
        sx_l, sy_l = to_slot(sx, sy)
        
        self.get_logger().info(f"📍 Stage-1 목표를 시작점으로 사용 (map): ({sx:.2f}, {sy:.2f})")
        self.get_logger().info(f"📍 시작 위치 (슬롯): ({sx_l:.2f}, {sy_l:.2f})")

        # 슬롯 크기(폭/길이) 추정
        if self._parking_area_idx is not None:
            Lslot, Wslot = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
        else:
            Lslot, Wslot = 5.0, 2.5
            
        self.get_logger().info(f"📍 슬롯 크기: {Lslot:.2f}m x {Wslot:.2f}m")

        # 1) '벽 라인'을 y = +wall_clear 로 가정.
        #    시작점이 도로 반대쪽(y<0)이면 부호 뒤집어 자동 보정.
        sign = 1.0 if sy_l >= 0.0 else -1.0
        wall_y = sign * abs(wall_clear)
        
        self.get_logger().info(f"📍 벽 라인: y = {wall_y:.2f} (부호: {sign})")

        # 2) 템플릿 포인트 (슬롯 프레임)
        # A: 시작 y를 벽 라인으로 부드럽게 투영 (x는 유지)
        A = (sx_l, wall_y)
        # B: 직선으로 -x 방향으로 내려오기
        B = (sx_l - abs(straight_len), wall_y)

        # 목표점 G: 슬롯 안쪽으로 약간 들어간 지점 (슬롯 중심 기준)
        #   x는 중심에서 뒤(goal_back)로, y는 벽 반대쪽으로 inside_m 만큼 안쪽
        G = (-abs(goal_back), -sign * (Wslot*0.5 - inside_m))
        
        self.get_logger().info(f"📍 템플릿 포인트:")
        self.get_logger().info(f"   - A (벽 붙기): ({A[0]:.2f}, {A[1]:.2f})")
        self.get_logger().info(f"   - B (벽 따라): ({B[0]:.2f}, {B[1]:.2f})")
        self.get_logger().info(f"   - G (슬롯 진입): ({G[0]:.2f}, {G[1]:.2f})")

        # 3) Cubic Bezier B -> G (S 느낌을 주는 제어점 배치)
        #    - 시작 접선: -x 방향, 끝 접선: +x 방향
        C1 = (B[0] - c1_len, B[1])         # 시작쪽 컨트롤(뒤로)
        C2 = (G[0] + c2_len, G[1])         # 끝쪽 컨트롤(앞으로)
        
        self.get_logger().info(f"📍 베지어 제어점:")
        self.get_logger().info(f"   - C1: ({C1[0]:.2f}, {C1[1]:.2f})")
        self.get_logger().info(f"   - C2: ({C2[0]:.2f}, {C2[1]:.2f})")

        def bezier(t):
            u = 1.0 - t
            bx = (u**3)*B[0] + 3*(u**2)*t*C1[0] + 3*u*(t**2)*C2[0] + (t**3)*G[0]
            by = (u**3)*B[1] + 3*(u**2)*t*C1[1] + 3*u*(t**2)*C2[1] + (t**3)*G[1]
            return bx, by

        # 4) 샘플링: start→A (직선), A→B (직선), B→G (베지어)
        path = Path()
        path.header = parking_pose.header
        
        def append_line(P, Q):
            dx, dy = Q[0]-P[0], Q[1]-P[1]
            L = max(1e-6, math.hypot(dx, dy))
            n = max(1, int(L / max(path_res, 1e-3)))
            for i in range(1, n+1):
                t = i / n
                xl, yl = P[0] + t*dx, P[1] + t*dy
                xm, ym = to_map(xl, yl)
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = xm
                pose.pose.position.y = ym
                # 방향(옵션)
                yaw = math.atan2(dy, dx) + yaw_slot
                pose.pose.orientation = yaw_to_quaternion(yaw)
                path.poses.append(pose)

        # start_l -> A
        self.get_logger().info("🔄 start → A (벽 붙기) 경로 생성")
        append_line((sx_l, sy_l), A)
        
        # A -> B
        self.get_logger().info("🔄 A → B (벽 따라 직선) 경로 생성")
        append_line(A, B)
        
        # B -> G (bezier)
        self.get_logger().info("🔄 B → G (베지어 S-curve) 경로 생성")
        # t 간격은 hc_ds로 길이 근사하여 잡음
        # 먼저 대략 길이 추정
        N_b = max(10, int(abs(B[0]-G[0]) / max(ds, 1e-3)))
        prev = B
        for i in range(1, N_b+1):
            t = i / N_b
            xl, yl = bezier(t)
            # 헤딩 계산용(미분 근사)
            dx, dy = xl - prev[0], yl - prev[1]
            xm, ym = to_map(xl, yl)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = xm
            pose.pose.position.y = ym
            yaw = math.atan2(dy, dx) + yaw_slot
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
            prev = (xl, yl)

        # 5) 간단 안전 체크(콘)
        self.get_logger().info("🚗 충돌 검사 중...")
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin
        
        collision_detected = False
        for i, p in enumerate(path.poses):
            if self._min_distance_to_cones(p.pose.position.x, p.pose.position.y) < eff_radius:
                self.get_logger().warn(f"Hardcoded path: cone too close at point {i}. Tune wall_clear/straight_len/inside_margin.")
                collision_detected = True
                break
                
        if not collision_detected:
            self.get_logger().info("✅ 충돌 검사 통과")

        self.get_logger().info(f"✅ 하드코딩 S-curve 경로 생성 완료: {len(path.poses)}개 점")
        self.get_logger().info("🔄 === 하드코딩 S-curve 경로 생성 종료 ===")
        return path if path.poses else None

    def _compute_stage2_path_oldlogic(self,
                                      stage1_goal: PoseStamped,
                                      stage2_goal: PoseStamped,
                                      parking_pose: PoseStamped) -> Optional[Path]:
        """
        단순한 3개 직선 구간 경로 생성:
        Stage-1 골 → (반대방향 1.5m) → 구역 중심 → 주차 포즈 위치
        프레임: 입력/출력 모두 map.
        """
        try:
            # 0) 파라미터 로드
            pre_reverse = float(self.get_parameter('oldlogic_pre_reverse').value)
            step_lin = float(self.get_parameter('oldlogic_pre_straight').value)
            center_offset = float(self.get_parameter('oldlogic_center_offset').value)
            
            self.get_logger().info(f"🔧 단순 직선 경로 파라미터:")
            self.get_logger().info(f"   - 사전 후진 거리: {pre_reverse:.2f}m")
            self.get_logger().info(f"   - 직선 해상도: {step_lin:.2f}m")
            self.get_logger().info(f"   - 구역 중심 오프셋: {center_offset:.2f}m")

            # 1) 3개 핵심 포인트 (map 프레임)
            # p1: Stage-1 골
            p1 = (stage1_goal.pose.position.x, stage1_goal.pose.position.y)
            
            # p2: Stage-1 골에서 반대 방향으로 pre_reverse만큼 직진
            stage1_yaw = quaternion_to_yaw(stage1_goal.pose.orientation)
            reverse_dx = -pre_reverse * math.cos(stage1_yaw)
            reverse_dy = -pre_reverse * math.sin(stage1_yaw)
            p2 = (p1[0] + reverse_dx, p1[1] + reverse_dy)
            
            # p3: 구역 중심 (주차 위치에서 주차 방향으로 center_offset만큼 전진)
            parking_yaw = quaternion_to_yaw(parking_pose.pose.orientation)
            center_dx = center_offset * math.cos(parking_yaw)
            center_dy = center_offset * math.sin(parking_yaw)
            p3 = (parking_pose.pose.position.x + center_dx, parking_pose.pose.position.y + center_dy)
            
            # p4: 주차 포즈 위치 (방향 정보 제외, 위치만)
            p4 = (parking_pose.pose.position.x, parking_pose.pose.position.y)
            
            self.get_logger().info(f"📍 3개 직선 구간 포인트 (map 프레임):")
            self.get_logger().info(f"   - p1 (Stage-1골): ({p1[0]:.2f}, {p1[1]:.2f})")
            self.get_logger().info(f"   - p2 (반대방향): ({p2[0]:.2f}, {p2[1]:.2f})")
            self.get_logger().info(f"   - p3 (구역중심): ({p3[0]:.2f}, {p3[1]:.2f})")
            self.get_logger().info(f"   - p4 (주차위치): ({p4[0]:.2f}, {p4[1]:.2f})")

            # 2) 직선 구간 생성 함수
            def linseg(a, b, step):
                ax, ay = a; bx, by = b
                L = math.hypot(bx-ax, by-ay)
                n = max(1, int(L/max(step,1e-3)))
                xs = np.linspace(ax, bx, n+1)
                ys = np.linspace(ay, by, n+1)
                return xs, ys

            # 3) 3개 직선 구간 생성
            path_x_l, path_y_l = [], []

            # 구간 1: p1 → p2 (Stage-1 골에서 반대방향으로 1.5m)
            self.get_logger().info("🔄 구간 1: Stage-1 골 → 반대방향 1.5m")
            xs, ys = linseg(p1, p2, step_lin)
            path_x_l += xs.tolist()
            path_y_l += ys.tolist()
            
            # 구간 2: p2 → p3 (반대방향 지점에서 구역 중심으로)
            self.get_logger().info("🔄 구간 2: 반대방향 지점 → 구역 중심")
            xs, ys = linseg(p2, p3, step_lin)
            # 중복점 제거
            if len(path_x_l)>0 and len(xs)>0 and (path_x_l[-1]==xs[0] and path_y_l[-1]==ys[0]):
                xs = xs[1:]; ys = ys[1:]
            path_x_l += xs.tolist()
            path_y_l += ys.tolist()

            # 구간 3: p3 → p4 (구역 중심에서 주차 위치로)
            self.get_logger().info("🔄 구간 3: 구역 중심 → 주차 위치")
            xs, ys = linseg(p3, p4, step_lin)
            # 중복점 제거
            if len(path_x_l)>0 and len(xs)>0 and (path_x_l[-1]==xs[0] and path_y_l[-1]==ys[0]):
                xs = xs[1:]; ys = ys[1:]
            path_x_l += xs.tolist()
            path_y_l += ys.tolist()

            # 4) Path 메시지 생성 (방향 정보는 계산용으로만 사용)
            self.get_logger().info("🔄 Path 메시지 생성 중...")
            path = Path()
            path.header = stage2_goal.header
            
            for i in range(len(path_x_l)):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = path_x_l[i]
                ps.pose.position.y = path_y_l[i]
                ps.pose.position.z = 0.0
                
                # 방향: 다음 점을 향하는 방향 (계산 편의용)
                if i < len(path_x_l)-1:
                    dx = path_x_l[i+1] - path_x_l[i]
                    dy = path_y_l[i+1] - path_y_l[i]
                    hd = math.atan2(dy, dx)
                else:
                    # 마지막 점은 주차 공간 방향 사용
                    hd = quaternion_to_yaw(parking_pose.pose.orientation)
                
                ps.pose.orientation = yaw_to_quaternion(hd)
                path.poses.append(ps)

            self.get_logger().info(f"✅ 단순 3개 직선 경로 생성 완료: {len(path.poses)}개 점")
            self.get_logger().info("🔄 === 단순 직선 경로 생성 종료 ===")
            return path if path.poses else None

        except Exception as e:
            self.get_logger().error(f'Simple straight-line Stage2 path failed: {e}')
            return None


def main(args=None) -> None:
    """메인 함수: ROS2 노드 실행"""
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
