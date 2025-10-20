#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ObstacleArray, Obstacle
from std_msgs.msg import Int32, Int64, Bool
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
# 이 파일은 자율주행 자동차의 2단계 주차 시스템을 구현합니다.
# - Stage-1: 주차 공간 앞으로 전진 이동
# - Stage-2: 주차 공간으로 후진 (3개 직선 구간 경로)
# 
# 주요 특징:
# - 콘 기반 충돌 회피
# - 실시간 경로 계획 및 발행


class PlannerNode(Node):
    """2단계 주차 계획 시스템 (Stage-1, Stage-2)

    이 노드는 자율주행 자동차가 주차 공간을 찾아 안전하게 주차하는 경로를 계획합니다.
    
    구독 토픽:
        /parking/parking_pose     (PoseStamped)    - 주차 공간 위치 정보
        /parking/cones_mapped_in_roi (ObstacleArray) - 안정화된 콘 정보
        /autocar/location         (Odometry)       - 차량 현재 위치 [필수]
    
    발행 토픽:
        /waypoints                (Path)           - 현재 활성 스테이지의 경로
        /waypoints_points         (PoseArray)      - 현재 활성 스테이지의 웨이포인트
        /reverse_flag             (Bool)           - 후진 모드 플래그
        /parking_stop_flag        (Bool)           - 주차 완료 3초 정지 플래그
        /parking_complete_flag    (Bool)           - 주차 종료 플래그
    """

    def __init__(self) -> None:
        super().__init__('planner_node')

        # ==================== 주차 계획 파라미터 ====================
        
        # ==================== 공통 파라미터 ====================
        # 차량 물리적 특성
        self.declare_parameter('vehicle_width', 1.16)           # 차량 폭 [m]
        self.declare_parameter('vehicle_length', 2.02)          # 차량 길이 [m]
        self.declare_parameter('safety_margin', 0.0)            # 충돌 회피를 위한 안전 여유 [m]
        self.declare_parameter('path_resolution', 0.3)         # 경로 점들 사이의 간격 [m]
        
        # 주차 공간 기본 정보
        self.declare_parameter('default_slot_len', 5.0)         # 주차 공간 길이 기본값 [m]
        self.declare_parameter('default_slot_width', 2.5)       # 주차 공간 폭 기본값 [m]
        
        # ==================== Stage-1 (전진 이동) 파라미터 ====================
        self.declare_parameter('front_margin', 5.0)               # 주차 공간 전방 경계에서 추가 여유 [m]
        self.declare_parameter('clear_lateral', 1.3)            # 주차 공간 옆쪽(도로쪽) 여유 [m]
        self.declare_parameter('yaw_offset_deg', 0.0)           # Stage-1에서 좌측으로 선회할 각도 [도]
        self.declare_parameter('show_stage1_path', True)        # Stage-1 경로 시각화 여부
        
        # ==================== Stage-2 목표 계산 파라미터 ====================
        self.declare_parameter('stage2_use_map_y_offset', False)  # Stage-2 목표를 map 프레임 y 오프셋으로 설정
        self.declare_parameter('stage2_goal_offset_y', 0.0)     # map 프레임 y축 오프셋 [m]
        self.declare_parameter('stage2_back_offset', 0.0)       # 슬롯 진행축 기준 뒤로 이동 거리 [m] (대체 옵션)
        self.declare_parameter('stage2_inside_margin', 0.2)     # 주차 공간 내부 안전 여유 [m]

        
        # ==================== Stage-2 경로 생성 우선순위별 파라미터 ====================
        
        # === 1순위: 작년 로직 (Old-logic) - 3개 직선 구간 경로 ===
        self.declare_parameter('oldlogic_use', True)            # 작년 로직 사용 여부 (최우선)
        self.declare_parameter('oldlogic_pre_reverse', 3.0)     # Stage-1 골에서 반대방향 직진 거리 [m]
        self.declare_parameter('oldlogic_pre_straight', 0.2)    # 초기 직선 step 크기 [m]
        self.declare_parameter('oldlogic_center_offset', 2.5)   # 구역 중심 오프셋 (주차위치에서 앞으로) [m]
        
        # ==================== 스테이지 제어 파라미터 ====================
        self.declare_parameter('auto_advance', True)            # 오도메트리 기반 자동 스테이지 전환 여부
        self.declare_parameter('test_mode_immediate_s_curve', False)  # 테스트 모드: 주차 포즈 수신 시 즉시 S자 경로 생성
        self.declare_parameter('stage_position_tolerance', 2.0)  # 스테이지 완료 위치 허용 오차 [m]
        self.declare_parameter('stage_yaw_tolerance_deg', 100.0)    # 스테이지 완료 방향 허용 오차 [도]
        self.declare_parameter('publish_unified_waypoints', False)  # 현재 스테이지만 /waypoints 퍼블리시 여부
        
        # ==================== 주차 완료 파라미터 ====================
        self.declare_parameter('parking_stop_duration', 3.0)    # 주차 완료 시 정지 시간 [초]
        self.declare_parameter('parking_complete_yaw_tolerance_deg', 5.0)  # 주차 완료 방향 임계치 [도]
        
        # ==================== 좌표계 설정 ====================
        self.declare_parameter('frame_id', 'map')  # 기본 좌표계

        # ==================== 상태 변수 ====================
        # 주차 관련 상태
        self._parking_pose: Optional[PoseStamped] = None      # 주차 공간 위치
        self._odom: Optional[Odometry] = None                 # 차량 현재 위치 (오도메트리)
        self._stable_cones: List[Obstacle] = []               # 안정화된 콘 정보 (충돌 회피용)
        self._current_lanelet_id: Optional[int] = None        # 현재 차량이 위치한 Lanelet ID
        self._parking_zone_ids: List[int] = [21]             # 주차 구역 ID (K-City 본선 기준)
        
        # 각 스테이지별 목표 위치 저장
        self._last_stage1_goal: Optional[PoseStamped] = None  # Stage-1 목표 위치
        self._last_stage2_goal: Optional[PoseStamped] = None  # Stage-2 목표 위치
        
        # 스테이지 제어
        self._stage: int = 1                                   # 현재 스테이지 (1, 2)
        
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
        self.create_subscription(PoseStamped, '/parking/parking_pose', self._on_parking_pose, 10)         # 주차 공간 위치
        self.create_subscription(ObstacleArray, '/parking/cones_mapped_in_roi', self._on_stable_cones, 10)  # 안정화된 콘 정보
        
        # 차량 위치 및 제어 관련 구독
        self.create_subscription(Odometry, '/autocar/location', self._on_odom, 20)                        # 차량 현재 위치 (오도메트리)
        self.create_subscription(Int64, '/current_lanelet_id', self._on_lanelet_id, 10)                   # 현재 Lanelet ID

        # 주차 포즈 계산 완료 플래그 발행
        self._parking_pose_ready_pub = self.create_publisher(Bool, '/parking/pose_ready', 10)

        # ==================== 타이머 설정 ====================
        self.create_timer(0.2, self._on_timer)  # 0.2초마다 스테이지 재평가 및 경로 발행

        self.get_logger().info('Planner node started (Stage-1 goal computation).')

    # ==================== 콜백 함수들 ====================
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

    def _on_lanelet_id(self, msg: Int64) -> None:
        """현재 Lanelet ID 수신 콜백"""
        self._current_lanelet_id = int(msg.data)
        self.get_logger().debug(f'현재 Lanelet ID: {self._current_lanelet_id}')

    def _is_in_parking_zone(self) -> bool:
        """현재 주차 구역에 있는지 확인"""
        return (self._current_lanelet_id is not None and 
                self._current_lanelet_id in self._parking_zone_ids)

    def _publish_parking_pose_ready(self) -> None:
        """주차 포즈 계산 완료 플래그 발행"""
        msg = Bool()
        msg.data = True
        self._parking_pose_ready_pub.publish(msg)
        self.get_logger().info('주차 포즈 계산 완료 플래그 발행')

    # ==================== 핵심 로직 ====================
    def _on_timer(self) -> None:
        """주기적으로 호출되는 메인 루프 (0.2초마다)
        주차 경로 생성 및 발행
        """
        self._maybe_publish_goal()

    def _transform_utm_to_map(self, utm_pose: Odometry) -> Optional[PoseStamped]:
        """UTM 좌표를 map 프레임으로 변환
        Args:
            utm_pose: UTM 좌표계의 pose (world 프레임)
        Returns:
            map 프레임의 PoseStamped 또는 None (변환 실패 시)
        """
        try:
            # 1. world → map static transform 조회 (map 원점의 UTM 좌표)
            try:
                world_to_map = self._tf_buffer.lookup_transform(
                    'map',
                    'world',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.get_logger().debug(f"World→Map static transform 조회 성공")
            except Exception as e:
                self.get_logger().error(f"World→Map transform 조회 실패: {e}")
                return None

            # 2. UTM 좌표에서 map 원점을 빼서 map 프레임 상대 좌표 계산
            map_origin_x = world_to_map.transform.translation.x
            map_origin_y = world_to_map.transform.translation.y
            
            vehicle_utm_x = utm_pose.pose.pose.position.x
            vehicle_utm_y = utm_pose.pose.pose.position.y
            
            vehicle_map_x = vehicle_utm_x + map_origin_x
            vehicle_map_y = vehicle_utm_y + map_origin_y

            # 3. map 프레임의 PoseStamped 생성
            map_pose = PoseStamped()
            map_pose.header = utm_pose.header
            map_pose.header.frame_id = 'map'
            map_pose.pose.position.x = vehicle_map_x
            map_pose.pose.position.y = vehicle_map_y
            map_pose.pose.position.z = 0.0
            map_pose.pose.orientation = utm_pose.pose.pose.orientation

            # 디버깅용 로그
            self.get_logger().debug(
                f"\n[UTM→Map 변환]"
                f"\n  UTM 좌표: ({vehicle_utm_x:.2f}, {vehicle_utm_y:.2f})"
                f"\n  Map 원점(UTM): ({map_origin_x:.2f}, {map_origin_y:.2f})"
                f"\n  Map 좌표: ({vehicle_map_x:.2f}, {vehicle_map_y:.2f})"
            )

            return map_pose

        except Exception as e:
            self.get_logger().error(f"UTM→Map 변환 중 예상치 못한 오류 발생: {e}")
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

    def _compute_stage1_goal(self, parking_pose: PoseStamped) -> PoseStamped:
        """Stage-1 목표 위치 계산 (주차 공간 앞으로 전진 이동)
        Args:
            parking_pose: 주차 공간 위치
        Returns:
            목표 위치
        """
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        center = (parking_pose.pose.position.x, parking_pose.pose.position.y)

        # 주차 공간 크기 (고정값)
        L, W = 5.0, 2.5

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

        return goal

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
        transformed_pose = self._transform_utm_to_map(self._odom)
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
        
        # Stage-2 완료 시 주차 완료 (Stage-3 없음)
        elif self._stage == 2 and self._last_stage2_goal is not None:
            distance = math.hypot(
                current_pos.x - self._last_stage2_goal.pose.position.x,
                current_pos.y - self._last_stage2_goal.pose.position.y
            )
            if distance < tolerance:
                self.get_logger().info(f"Stage-2 완료 - 주차 완료 (거리={distance:.2f}m < {tolerance}m)")
        
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
        transformed_pose = self._transform_utm_to_map(self._odom)
        if transformed_pose is None:
            return False
            
        # 방향 임계치 확인
        yaw_tolerance = math.radians(float(self.get_parameter('parking_complete_yaw_tolerance_deg').value))
        current_yaw = quaternion_to_yaw(transformed_pose.pose.orientation)
        parking_yaw = quaternion_to_yaw(self._parking_pose.pose.orientation)
        yaw_diff = abs(wrap_to_pi(current_yaw - parking_yaw))
        
        self.get_logger().info(f"주차 완료 체크 - 현재 방향: {math.degrees(current_yaw):.1f}°, 주차 방향: {math.degrees(parking_yaw):.1f}°, 차이: {math.degrees(yaw_diff):.1f}°")
        
        # Stage-2에서 주차 위치 도달 확인
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
        # 주차 구역에 있는지 확인
        if not self._is_in_parking_zone():
            return

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

        # Stage-1 목표 계산
        try:
            stage1_goal = self._compute_stage1_goal(self._parking_pose)
            self._last_stage1_goal = stage1_goal
            # 주차 포즈 계산 완료 플래그 발행
            self._publish_parking_pose_ready()
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
            
            # Stage-2 목표 계산
            self.get_logger().info("Stage-2 목표 계산 중...")
            stage2_goal = self._compute_stage2_goal(self._parking_pose)
            if stage2_goal is not None:
                self._last_stage2_goal = stage2_goal
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
                
                if s_curve_enabled and current_path is None:
                    self.get_logger().warn("⚠️ 기존 S자 경로 로직이 제거되어 폴백 없음")
                else:
                    self.get_logger().error("❌ S자 경로가 비활성화됨 - Stage-2 경로 생성 불가")
                
                if current_path is not None:
                    self.get_logger().info(f"📤 Stage-2 경로 발행 완료: {len(current_path.poses)}개 점")
                else:
                    self.get_logger().error("❌ Stage-2 모든 경로 생성 실패")
            else:
                self.get_logger().error("❌ Stage-2 목표 계산 실패")
            self.get_logger().info("=== Stage-2 디버그 종료 ===")
        
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

    def _is_near_pose(self, a: PoseStamped, b: PoseStamped, pos_tol: float, yaw_tol: float) -> bool:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dist = math.hypot(dx, dy)
        ya = quaternion_to_yaw(a.pose.orientation)
        yb = quaternion_to_yaw(b.pose.orientation)
        dyaw = abs(wrap_to_pi(ya - yb))
        return (dist <= pos_tol) and (dyaw <= yaw_tol)

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
        transformed_pose = self._transform_utm_to_map(current_pose)
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
