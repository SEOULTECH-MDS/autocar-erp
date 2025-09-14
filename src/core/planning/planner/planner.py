#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ObstacleArray, Obstacle, ModeState
from std_msgs.msg import Int32
import tf2_ros
import tf2_geometry_msgs


def wrap_to_pi(angle: float) -> float:
    """Wrap any angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    """Extract yaw from geometry_msgs/Quaternion (Z-W only)."""
    return math.atan2(2.0 * q.z * q.w, 1.0 - 2.0 * q.z * q.z)


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


# Segment 클래스 제거 - 가상 벽 대신 콘 기반 충돌 회피 사용


class PlannerNode(Node):
    """Stage-1 parking planner.

    - Subscribes:
        /open_slot_pose (PoseStamped)
        /virtual_walls   (planning_msgs/ObstacleArray)
        /autocar/location (nav_msgs/Odometry) [required]
    - Publishes:
        /waypoints               (nav_msgs/Path)  [MAIN]  활성 스테이지의 최신 경로를 통합 퍼블리시
        /stage1_path             (nav_msgs/Path)  [DEBUG] 스테이지1 경로 시각화/디버그용
        /stage2_path             (nav_msgs/Path)  [DEBUG] 스테이지2 경로 시각화/디버그용
        /stage3_path             (nav_msgs/Path)  [DEBUG] 스테이지3 경로 시각화/디버그용
        /stage1_goal             (PoseStamped)    [DEBUG]
        /stage2_goal             (PoseStamped)    [DEBUG]
        /stage3_goal             (PoseStamped)    [DEBUG]
        /planner_debug_markers   (MarkerArray)    [DEBUG]
    """

    def __init__(self) -> None:
        super().__init__('planner_node')

        # Parameters
        self.declare_parameter('yaw_offset_deg', 30.0)      # 좌측 선회 각도
        self.declare_parameter('s_overshoot', 1.0)          # (구버전) 슬롯 중심을 얼마나 지나칠지 [m]
        self.declare_parameter('front_margin', 1.0)         # 위쪽(전방) 경계선에서 추가 전방 여유 [m]
        self.declare_parameter('clear_lateral', 2.0)        # 도로쪽 여유 [m]
        self.declare_parameter('default_slot_len', 5.0)     # 추정 실패 시 기본 L
        self.declare_parameter('default_slot_width', 2.5)   # 추정 실패 시 기본 W
        self.declare_parameter('angle_eps_deg', 5.0)       # 평행/수직 분류 각 허용치
        self.declare_parameter('prefer_near_open_slot', True)
        self.declare_parameter('near_radius', 8.0)
        # Road centerline based lateral placement
        self.declare_parameter('prefer_centerline_lateral', True)
        self.declare_parameter('centerline_left_margin', 0.1)  # 도로 중심선 기준 왼쪽(+u_lat)으로 추가 여유
        # Turning and path generation
        self.declare_parameter('turn_radius', 0.5)            # 좌회전 원호 반경 [m]
        self.declare_parameter('path_resolution', 0.01)        # 경로 샘플링 간격 [m]
        self.declare_parameter('vehicle_width', 1.16)         # [m]
        self.declare_parameter('vehicle_length', 2.02)        # [m]
        self.declare_parameter('safety_margin', 0.1)          # [m]
        self.declare_parameter('show_stage1_path', True)     # 시각화용: 기본 비활성
        # Stage-2 goal rule
        self.declare_parameter('stage2_use_map_y_offset', True)
        self.declare_parameter('stage2_goal_offset_y', 0.0)  # map 프레임 y에서 -1m
        self.declare_parameter('stage2_back_offset', 0.0)     # 슬롯 진행축 기준 뒤로 1m (fallback)
        self.declare_parameter('stage2_straight_only', True)  # 원호 없이 직선 경로만 생성
        # Stage-2 guided reverse (no explicit goal pose)
        self.declare_parameter('stage2_guided', True)
        self.declare_parameter('stage2_inside_margin', 0.2)
        self.declare_parameter('stage2_k_lat', 0.8)
        self.declare_parameter('stage2_k_yaw', 0.8)
        self.declare_parameter('stage2_max_steps', 800)
        self.declare_parameter('stage2_lookahead', 1.0)
        # Stage-3
        self.declare_parameter('stage3_preview', True)
        # General stage control
        self.declare_parameter('auto_advance', True)            # 오돔 기반 자동 스테이지 전환
        # self.declare_parameter('prefer_odom', True)             # 더 이상 사용하지 않음 - location만 사용
        self.declare_parameter('stage_position_tolerance', 0.25)  # [m]
        self.declare_parameter('stage_yaw_tolerance_deg', 8.0)    # [deg]
        self.declare_parameter('publish_unified_waypoints', False)  # 현재 스테이지만 /waypoints(및 points) 퍼블리시
        # Delivery mission
        self.declare_parameter('delivery_position_tolerance', 0.3)
        self.declare_parameter('delivery_yaw_tolerance_deg', 12.0)
        self.declare_parameter('delivery_stop_offset', 1.0)  # 표지판 앞 정지 오프셋 [m]
        self.declare_parameter('delivery_auto_activate', False)  # delivery 토픽 수신 시 자동 전환 (기본 OFF)
        self.declare_parameter('enable_delivery_planning', False)  # 배달 경로 생성 전체 활성화 토글 (기본 OFF)
        # Frames
        self.declare_parameter('frame_id', 'map')

        # State
        self._parking_pose: Optional[PoseStamped] = None
        self._odom: Optional[Odometry] = None
        # 주차 패키지에서 받은 정보들
        self._parking_area_idx: Optional[int] = None
        self._stable_cones: List[Obstacle] = []
        self._last_stage1_goal: Optional[PoseStamped] = None
        self._last_stage2_goal: Optional[PoseStamped] = None
        self._last_stage3_goal: Optional[PoseStamped] = None
        # Stage machine: 1,2,3
        self._stage: int = 1
        # External selector (overrides if provided)
        self._selector_stage: Optional[int] = None
        # Mission mode: 'parking' | 'delivery'
        self._mission: str = 'parking'
        # Delivery-specific state
        self._delivery_spots: Optional[PoseArray] = None
        self._delivery_target_sign: Optional[int] = None  # 3,4,5 -> B1,B2,B3
        self._delivery_phase: str = 'pickup'  # 'pickup' then 'dropoff'
        self._delivery_pick_target: Optional[PoseStamped] = None
        self._delivery_drop_target: Optional[PoseStamped] = None
        
        # TF Buffer for coordinate transformation
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Publishers
        # MAIN topic: 컨트롤 파트가 구독할 표준 토픽 (활성 스테이지 최신 경로)
        self._waypoints_pub = self.create_publisher(Path, '/waypoints', 10)
        self._waypoints_points_pub = self.create_publisher(PoseArray, '/waypoints_points', 10)

        # DEBUG topics: 스테이지별 Path/Goal 및 시각화 마커
        self._goal_pub = self.create_publisher(PoseStamped, '/stage1_goal', 10)
        self._stage2_goal_pub = self.create_publisher(PoseStamped, '/stage2_goal', 10)
        self._vis_pub = self.create_publisher(MarkerArray, '/planner_debug_markers', 10)
        self._path_pub = self.create_publisher(Path, '/stage1_path', 10)
        self._stage2_path_pub = self.create_publisher(Path, '/stage2_path', 10)
        self._stage3_path_pub = self.create_publisher(Path, '/stage3_path', 10)
        self._stage3_goal_pub = self.create_publisher(PoseStamped, '/stage3_goal', 10)
        # Stage-wise waypoints (PoseArray) publishing
        self._stage1_points_pub = self.create_publisher(PoseArray, '/stage1_waypoints', 10)
        self._stage2_points_pub = self.create_publisher(PoseArray, '/stage2_waypoints', 10)
        self._stage3_points_pub = self.create_publisher(PoseArray, '/stage3_waypoints', 10)
        # Delivery mission publishers
        self._delivery_pick_path_pub = self.create_publisher(Path, '/delivery_pick_path', 10)
        self._delivery_drop_path_pub = self.create_publisher(Path, '/delivery_drop_path', 10)
        self._delivery_pick_points_pub = self.create_publisher(PoseArray, '/delivery_pick_waypoints', 10)
        self._delivery_drop_points_pub = self.create_publisher(PoseArray, '/delivery_drop_waypoints', 10)

        # Subscribers
        # 주차 패키지 출력 구독
        self.create_subscription(Int32, '/parking/open_area_idx', self._on_parking_area_idx, 10)
        self.create_subscription(PoseStamped, '/parking/parking_pose', self._on_parking_pose, 10)
        self.create_subscription(ObstacleArray, '/parking/cones_mapped_in_roi', self._on_stable_cones, 10)
        # 기존 구독자들
        # Odom for stage completion/starting pose
        self.create_subscription(Odometry, '/autocar/location', self._on_odom, 20)
        # Optional stage selector interface (1,2,3)
        self.create_subscription(Int32, '/stage_selector', self._on_stage_selector, 10)
        # Mission selection and delivery inputs
        self.create_subscription(ModeState, '/mode_state', self._on_mode_state, 10)
        self.create_subscription(PoseArray, '/deliverysign_spot', self._on_delivery_spots, 10)
        self.create_subscription(Int32, '/target_sign', self._on_target_sign, 10)
        # Preferred: explicit targets from mode selector (map frame)
        self.create_subscription(PoseStamped, '/delivery_pick_target', self._on_delivery_pick_target, 10)
        self.create_subscription(PoseStamped, '/delivery_drop_target', self._on_delivery_drop_target, 10)

        # Timer to re-evaluate stage and publish paths
        self.create_timer(0.2, self._on_timer)

        self.get_logger().info('Planner node started (Stage-1 goal computation).')

    # ───────────────────────────── Callbacks ─────────────────────────────
    def _on_parking_area_idx(self, msg: Int32) -> None:
        """주차 공간 인덱스 수신"""
        self._parking_area_idx = int(msg.data)
        self.get_logger().info(f'Parking area idx: {self._parking_area_idx}')
        self._maybe_publish_goal()

    def _on_parking_pose(self, msg: PoseStamped) -> None:
        """주차 포즈 수신"""
        self._parking_pose = msg
        self.get_logger().info(f'Parking pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self._maybe_publish_goal()

    def _on_stable_cones(self, msg: ObstacleArray) -> None:
        """안정화된 콘 정보 수신"""
        self._stable_cones = msg.obstacles
        self.get_logger().info(f'Received {len(self._stable_cones)} stable cones')
        self._maybe_publish_goal()


    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        # 스테이지 전환 체크를 위해 _maybe_publish_goal 호출
        self._maybe_publish_goal()

    def _on_stage_selector(self, msg: Int32) -> None:
        val = int(msg.data)
        if val in (1, 2, 3):
            self._selector_stage = val
            self._stage = val
            self.get_logger().info(f'Stage overridden by selector: {self._stage}')
            self._maybe_publish_goal()

    def _on_mode_state(self, msg: ModeState) -> None:
        try:
            if int(msg.current_mode) == int(ModeState.DELIVERY):
                if self._mission != 'delivery':
                    self.get_logger().info('Mission switched to DELIVERY')
                self._mission = 'delivery'
            elif int(msg.current_mode) == int(ModeState.PARKING):
                if self._mission != 'parking':
                    self.get_logger().info('Mission switched to PARKING')
                self._mission = 'parking'
        except Exception:
            pass

    def _on_delivery_spots(self, msg: PoseArray) -> None:
        """센서퓨전에서 수신한 표지판 후보 좌표 배열.
        - frame_id를 map으로 강제하여 일관성 유지.
        - delivery_auto_activate=True일 경우, 이 토픽 수신만으로도 배달 미션으로 전환.
        참고: enable_delivery_planning=False면 실제 경로 생성은 수행하지 않음.
        """
        # Force header to map for consistency
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        self._delivery_spots = msg
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('Auto-activated DELIVERY mission (spots received)')

    def _on_target_sign(self, msg: Int32) -> None:
        """모드 셀렉터/인지 노드에서 목표 B 표지판 식별자를 수신(3=B1, 4=B2, 5=B3).
        - delivery_auto_activate=True일 때 배달 미션으로 자동 전환만 수행.
        - 실제 목표 위치는 pick/drop 타겟 토픽 또는 spots로 보완됨.
        """
        try:
            self._delivery_target_sign = int(msg.data)
        except Exception:
            self._delivery_target_sign = None
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('Auto-activated DELIVERY mission (target sign received)')

    def _on_delivery_pick_target(self, msg: PoseStamped) -> None:
        """모드 셀렉터가 산출한 상차 타겟 표지판 위치.
        - 항상 map 프레임으로 강제 저장.
        - delivery_auto_activate=True면 이 토픽만으로 배달 미션 활성화.
        """
        # enforce map frame
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        self._delivery_pick_target = msg
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('Auto-activated DELIVERY mission (pick target)')

    def _on_delivery_drop_target(self, msg: PoseStamped) -> None:
        """모드 셀렉터가 산출한 하차 타겟 표지판 위치.
        - 항상 map 프레임으로 강제 저장.
        - delivery_auto_activate=True면 이 토픽만으로 배달 미션 활성화.
        """
        # enforce map frame
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        self._delivery_drop_target = msg
        if bool(self.get_parameter('delivery_auto_activate').value) and self._mission != 'delivery':
            self._mission = 'delivery'
            self.get_logger().info('Auto-activated DELIVERY mission (drop target)')

    # ───────────────────────────── Core Logic ────────────────────────────
    def _on_timer(self) -> None:
        """주기적으로 호출되는 메인 루프.
        - enable_delivery_planning=True 이고 미션이 delivery일 때만 배달 경로 생성.
        - 그 외에는 주차(파킹) 경로 생성 로직 수행.
        """
        if bool(self.get_parameter('enable_delivery_planning').value) and self._mission == 'delivery':
            self._maybe_publish_delivery()
        else:
            self._maybe_publish_goal()
    def _get_slot_dimensions_from_area_idx(self, area_idx: int) -> Tuple[float, float]:
        """모든 슬롯은 동일한 크기: 길이 5m, 폭 2.5m"""
        return 5.0, 2.5  # 모든 슬롯 동일

    def _transform_odom_to_map(self, odom: Odometry) -> Optional[PoseStamped]:
        """Odometry를 map 프레임으로 변환"""
        try:
            # Odometry를 PoseStamped로 변환
            pose_stamped = PoseStamped()
            pose_stamped.header = odom.header
            pose_stamped.pose = odom.pose.pose
            
            # map 프레임으로 변환
            transformed_pose = self._tf_buffer.transform(pose_stamped, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            return transformed_pose
        except Exception as e:
            self.get_logger().warn(f"TF transformation failed: {e}")
            return None

    def _min_distance_to_cones(self, x: float, y: float) -> float:
        """콘까지의 최소 거리 계산 (가상 벽 대체)"""
        if not self._stable_cones:
            return float('inf')
        
        min_dist = float('inf')
        for cone in self._stable_cones:
            dist = math.hypot(x - cone.center.x, y - cone.center.y) - cone.radius
            min_dist = min(min_dist, dist)
        
        return min_dist

    def _compute_stage1_goal(self, parking_pose: PoseStamped) -> Tuple[PoseStamped, MarkerArray]:
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        center = (parking_pose.pose.position.x, parking_pose.pose.position.y)

        # Dimensions from parking area idx
        if self._parking_area_idx is not None:
            L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
        else:
            L, W = 5.0, 2.5  # 기본값

        # Parameters
        # 목표는 "슬롯 위쪽 경계선까지 올라간 뒤" 정지
        # 전방 이동량 = L/2 (센터→전방 경계) + front_margin
        front_margin = float(self.get_parameter('front_margin').value)
        s_along = (L / 2.0) + front_margin
        clear_lat = float(self.get_parameter('clear_lateral').value)
        yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))

        # Unit vectors
        u_long = (math.cos(yaw_slot), math.sin(yaw_slot))
        u_lat = (-math.sin(yaw_slot), math.cos(yaw_slot))  # road side is +u_lat

        # Lateral placement: slot-based offset (centerline 계산 제거)
        lateral = (W / 2.0 + clear_lat)

        # Stop pose
        x_entry = center[0] + s_along * u_long[0] + lateral * u_lat[0]
        y_entry = center[1] + s_along * u_long[1] + lateral * u_lat[1]
        yaw_entry = wrap_to_pi(yaw_slot + yaw_offset)

        goal = PoseStamped()
        goal.header = parking_pose.header
        goal.pose.position.x = x_entry
        goal.pose.position.y = y_entry
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw_entry)

        # Visualization markers
        markers = MarkerArray()

        # 0: stop pose arrow
        arrow = Marker()
        arrow.header = goal.header
        arrow.ns = 'stage1_goal'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = goal.pose
        arrow.scale.x = 1.0  # arrow length
        arrow.scale.y = 0.2
        arrow.scale.z = 0.2
        arrow.color.a = 1.0
        arrow.color.r = 0.1
        arrow.color.g = 1.0
        arrow.color.b = 0.1
        markers.markers.append(arrow)

        # 1: slot center sphere
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

        # 2: lateral offset line (center -> goal)
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

    def _maybe_publish_goal(self) -> None:
        """모든 Stage의 경로를 계산하고 통합 waypoint로 발행"""
        # 입력 필요 확인
        if self._parking_pose is None:
            self.get_logger().warn("DEBUG: _parking_pose is None, returning")
            return
            
        # 시작 자세: odom 직접 사용
        if self._odom is None:
            self.get_logger().warn("DEBUG: _odom is None, returning")
            return
            
        # Location 변경 문제 해결: _odom을 로컬 변수로 복사
        current_odom = self._odom
        self.get_logger().info("DEBUG: Starting _maybe_publish_goal")
        self.get_logger().info(f"DEBUG: Using odom: ({current_odom.pose.pose.position.x:.2f}, {current_odom.pose.pose.position.y:.2f})")

        # Stage-1 goal/markers는 항상 갱신(디버그 가시화)
        try:
            stage1_goal, markers = self._compute_stage1_goal(self._parking_pose)
            self._last_stage1_goal = stage1_goal
            self._goal_pub.publish(stage1_goal)
            self._vis_pub.publish(markers)
        except Exception as e:
            self.get_logger().warn(f'Stage-1 goal computation failed: {e}')
            return

        # 스테이지 전환 체크
        stage_advanced = self._check_stage_advancement()
        if stage_advanced:
            self.get_logger().info(f"Stage advanced to: {self._stage}")
        
        # 현재 스테이지에 맞는 경로만 계산하고 퍼블리시
        current_path = None
        
        if self._stage == 1:
            # Stage 1 경로
            show_stage1 = bool(self.get_parameter('show_stage1_path').value)
            if show_stage1:
                self.get_logger().info("DEBUG: Computing stage1 path...")
                try:
                    current_path = self._compute_stage1_path(current_odom, stage1_goal)
                    if current_path is not None:
                        self._path_pub.publish(current_path)
                        self._publish_points(self._stage1_points_pub, current_path)
                        self.get_logger().info(f"Stage 1 path: {len(current_path.poses)} points")
                    else:
                        self.get_logger().warn("DEBUG: Stage1 path is None")
                except Exception as e:
                    self.get_logger().error(f"Stage 1 path computation failed: {e}")
                    import traceback
                    self.get_logger().error(f"Traceback: {traceback.format_exc()}")
        
        elif self._stage == 2:
            # Stage 2 골과 경로
            stage2_goal = self._compute_stage2_goal(self._parking_pose)
            if stage2_goal is not None:
                self._stage2_goal_pub.publish(stage2_goal)
                self.get_logger().info(f"Stage 2 goal published: ({stage2_goal.pose.position.x:.2f}, {stage2_goal.pose.position.y:.2f})")
                
                # Stage 2 경로 계산
                if bool(self.get_parameter('stage2_guided').value):
                    current_path = self._compute_stage2_path_guided(current_odom, self._parking_pose)
                else:
                    current_path = self._compute_stage2_path(current_odom, stage2_goal)
                
                if current_path is not None:
                    self._stage2_path_pub.publish(current_path)
                    self._publish_points(self._stage2_points_pub, current_path)
                    self.get_logger().info(f"Stage 2 path: {len(current_path.poses)} points")
            else:
                self.get_logger().warn("DEBUG: Stage 2 goal is None")
        
        elif self._stage == 3:
            # Stage 3 골과 경로
            stage3_goal = self._compute_stage3_goal(self._parking_pose)
            if stage3_goal is not None:
                self._stage3_goal_pub.publish(stage3_goal)
                self.get_logger().info(f"Stage 3 goal published: ({stage3_goal.pose.position.x:.2f}, {stage3_goal.pose.position.y:.2f})")
                yaw_slot = quaternion_to_yaw(self._parking_pose.pose.orientation)
                current_path = self._compute_stage3_path(current_odom, yaw_slot)
                
                if current_path is not None:
                    self._stage3_path_pub.publish(current_path)
                    self._publish_points(self._stage3_points_pub, current_path)
                    self.get_logger().info(f"Stage 3 path: {len(current_path.poses)} points")
            else:
                self.get_logger().warn("DEBUG: Stage 3 goal is None")
        
        # 현재 스테이지 경로를 /waypoints 토픽으로 퍼블리시
        if current_path is not None:
            self._waypoints_pub.publish(current_path)
            self._publish_points(self._waypoints_points_pub, current_path)
            self.get_logger().info(f"Published stage {self._stage} waypoints: {len(current_path.poses)} points")
        else:
            self.get_logger().warn(f"No path available for stage {self._stage}")
        
        # DEBUG: 모든 스테이지 골과 경로도 퍼블리시 (시각화용)
        self._publish_all_debug_paths(current_odom)
    
    def _publish_all_debug_paths(self, current_odom: Odometry) -> None:
        """모든 스테이지의 골과 경로를 DEBUG 토픽으로 퍼블리시 (시각화용)"""
        try:
            # Stage 1 골과 경로 (항상 퍼블리시)
            stage1_goal = self._compute_stage1_goal()
            if stage1_goal is not None:
                self._goal_pub.publish(stage1_goal)
                self._last_stage1_goal = stage1_goal
                
                show_stage1 = bool(self.get_parameter('show_stage1_path').value)
                if show_stage1:
                    stage1_path = self._compute_stage1_path(current_odom, stage1_goal)
                    if stage1_path is not None:
                        self._path_pub.publish(stage1_path)
                        self._publish_points(self._stage1_points_pub, stage1_path)
            
            # Stage 2 골과 경로
            if self._parking_pose is not None:
                stage2_goal = self._compute_stage2_goal(self._parking_pose)
                if stage2_goal is not None:
                    self._stage2_goal_pub.publish(stage2_goal)
                    self._last_stage2_goal = stage2_goal
                    
                    if bool(self.get_parameter('stage2_guided').value):
                        stage2_path = self._compute_stage2_path_guided(current_odom, self._parking_pose)
                    else:
                        stage2_path = self._compute_stage2_path(current_odom, stage2_goal)
                    
                    if stage2_path is not None:
                        self._stage2_path_pub.publish(stage2_path)
                        self._publish_points(self._stage2_points_pub, stage2_path)
                
                # Stage 3 골과 경로
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
            self.get_logger().warn(f"Debug paths publishing failed: {e}")

    # ───────────────────────────── Delivery Mission ──────────────────────
    def _maybe_publish_delivery(self) -> None:
        """배달 미션 경로 퍼블리시(상차→하차 단계).
        - 상차 단계: pick 타겟(있으면 우선)을 사용, 없으면 spots에서 진행방향 앞 최근접 선택.
        - 하차 단계: drop 타겟(있으면 우선)을 사용, 없으면 spots에서 진행방향 앞 최근접 선택.
        - 각 단계에서 표지판 앞 delivery_stop_offset [m] 위치에 정지 목표 생성 후 직선 경로 샘플링.
        - 완료 판정: 상차는 오프셋 목표 도달 시 dropoff로 전환. 하차는 target_sign이 설정된 뒤 오프셋 목표 도달 시 완료.
        주의: enable_delivery_planning=False면 이 함수는 호출되지 않음.
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
        # 실제 대회에서는 /autocar/location만 사용
        if self._odom is not None:
            ps = PoseStamped()
            ps.header = self._odom.header
            ps.pose = self._odom.pose.pose
            return ps
        return None

    def _publish_points(self, pub, path: Path) -> None:
        arr = PoseArray()
        arr.header = path.header
        for p in path.poses:
            pose = Pose()
            pose.position = p.pose.position
            pose.orientation = p.pose.orientation
            arr.poses.append(pose)
        pub.publish(arr)

    def _evaluate_and_advance_stage(self) -> bool:
        """오돔 기반으로 현재 스테이지 완료를 평가하고 필요 시 다음 스테이지로 전환.
        Returns True if stage was advanced."""
        if self._odom is None or self._parking_pose is None:
            return False
        pos_tol = float(self.get_parameter('stage_position_tolerance').value)
        yaw_tol = math.radians(float(self.get_parameter('stage_yaw_tolerance_deg').value))

        if self._stage == 1 and self._last_stage1_goal is not None:
            # Odometry를 PoseStamped로 변환
            current_pose = PoseStamped()
            current_pose.header = self._odom.header
            current_pose.pose = self._odom.pose.pose
            if self._is_near_pose(current_pose, self._last_stage1_goal, pos_tol, yaw_tol):
                self._stage = 2
                self.get_logger().info('Stage-1 complete -> Stage-2')
                return True

        elif self._stage == 2:
            if bool(self.get_parameter('stage2_guided').value):
                yaw_slot = quaternion_to_yaw(self._parking_pose.pose.orientation)
                center = (self._parking_pose.pose.position.x, self._parking_pose.pose.position.y)
                if self._parking_area_idx is not None:
                    L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
                else:
                    L, W = 5.0, 2.5
                margin = float(self.get_parameter('stage2_inside_margin').value)
                if self._inside_slot(self._odom.pose.pose.position.x, self._odom.pose.pose.position.y, center, yaw_slot, L, W, margin):
                    self._stage = 3
                    self.get_logger().info('Stage-2 (guided) complete -> Stage-3')
                    return True
            else:
                if self._last_stage2_goal is not None:
                    # Odometry를 PoseStamped로 변환
                    current_pose = PoseStamped()
                    current_pose.header = self._odom.header
                    current_pose.pose = self._odom.pose.pose
                    if self._is_near_pose(current_pose, self._last_stage2_goal, pos_tol, yaw_tol):
                        self._stage = 3
                        self.get_logger().info('Stage-2 complete -> Stage-3')
                        return True

        elif self._stage == 3:
            # 목표: 슬롯 중심에서 yaw 정렬
            g3 = self._last_stage3_goal or self._compute_stage3_goal(self._parking_pose)
            if g3 is not None:
                # Odometry를 PoseStamped로 변환
                current_pose = PoseStamped()
                current_pose.header = self._odom.header
                current_pose.pose = self._odom.pose.pose
                if self._is_near_pose(current_pose, g3, pos_tol, yaw_tol):
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
        """현재 위치에서 Stage 1 골까지 직선 경로"""
        if current_pose is None:
            self.get_logger().warn("DEBUG: current_pose is None in stage1 path")
            return None
        
        # Odometry를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(current_pose)
        if transformed_pose is None:
            self.get_logger().warn("DEBUG: Failed to transform odom to map frame")
            return None
        
        self.get_logger().info(f"DEBUG: Stage1 path - current (map): ({transformed_pose.pose.position.x:.2f}, {transformed_pose.pose.position.y:.2f})")
        self.get_logger().info(f"DEBUG: Stage1 path - goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        # Path message
        path = Path()
        path.header = goal.header
        
        # 현재 위치와 골 위치 (모두 map 프레임)
        x0 = transformed_pose.pose.position.x
        y0 = transformed_pose.pose.position.y
        x1 = goal.pose.position.x
        y1 = goal.pose.position.y
        
        # 직선 경로 생성
        dx = x1 - x0
        dy = y1 - y0
        distance = math.hypot(dx, dy)
        
        if distance < 1e-6:  # 거리가 너무 작으면
            self.get_logger().warn("DEBUG: Stage1 path distance too small")
            return None
        
        # 경로 해상도
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
            self.get_logger().warn('Stage-1 path may collide with cones. Consider increasing front_margin/clear_lateral.')
        
        self.get_logger().info(f"DEBUG: Stage1 path generated with {len(path.poses)} points")
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
        if start_pose is None or open_slot_pose is None:
            self.get_logger().warn("DEBUG: start_pose or open_slot_pose is None in stage2 guided")
            return None
        # Slot geometry
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        center = (open_slot_pose.pose.position.x, open_slot_pose.pose.position.y)
        if self._parking_area_idx is not None:
            L, W = self._get_slot_dimensions_from_area_idx(self._parking_area_idx)
        else:
            L, W = 5.0, 2.5  # 기본값

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

        # Initialize - UTM 좌표를 map 프레임으로 변환
        transformed_pose = self._transform_odom_to_map(start_pose)
        if transformed_pose is None:
            self.get_logger().warn("DEBUG: Failed to transform odom to map frame in stage2")
            return None
            
        x = transformed_pose.pose.position.x
        y = transformed_pose.pose.position.y
        yaw = quaternion_to_yaw(transformed_pose.pose.orientation)
        path = Path(); path.header = open_slot_pose.header

        # Stage 2 골을 먼저 계산해서 target으로 사용
        stage2_goal = self._compute_stage2_goal(open_slot_pose)
        if stage2_goal is None:
            self.get_logger().warn("DEBUG: Failed to compute stage2 goal")
            return None
        
        # Stage 2 골의 좌표를 target으로 사용
        target_world = (stage2_goal.pose.position.x, stage2_goal.pose.position.y)
        
        self.get_logger().info(f"DEBUG: Stage2 start - current: ({x:.2f}, {y:.2f}), target: ({target_world[0]:.2f}, {target_world[1]:.2f})")
        self.get_logger().info(f"DEBUG: Stage2 start - distance to target: {math.hypot(target_world[0] - x, target_world[1] - y):.2f}")

        # 직선 경로 생성 (Stage 1과 동일한 방식)
        x0, y0 = x, y
        dx = target_world[0] - x0
        dy = target_world[1] - y0
        distance = math.hypot(dx, dy)
        n_points = max(1, int(distance / ds))
        
        self.get_logger().info(f"DEBUG: Stage2 straight line - distance: {distance:.2f}, points: {n_points}")
        
        for i in range(n_points + 1):
            t = i / n_points
            px = x0 + t * dx
            py = y0 + t * dy
            
            # 충돌 검사
            width = float(self.get_parameter('vehicle_width').value)
            length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
            eff_radius = 0.5 * math.hypot(width, length) + safety_margin
            if self._min_distance_to_cones(px, py) < eff_radius:
                self.get_logger().warn(f'Stage-2 straight line would collide with cone at point {i}. Stopping early.')
                break
                
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(math.atan2(dy, dx))
            path.poses.append(pose)

        self.get_logger().info(f"DEBUG: Stage2 path generated with {len(path.poses)} points")
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
        if parking_pose is None:
            return None
        yaw_slot = quaternion_to_yaw(parking_pose.pose.orientation)
        center = (parking_pose.pose.position.x, parking_pose.pose.position.y)
        goal = PoseStamped()
        goal.header = parking_pose.header
        use_map = bool(self.get_parameter('stage2_use_map_y_offset').value)
        if use_map:
            offset_y = float(self.get_parameter('stage2_goal_offset_y').value)
            goal.pose.position.x = center[0]
            goal.pose.position.y = center[1] + offset_y
        else:
            back = float(self.get_parameter('stage2_back_offset').value)
            u_long = (math.cos(yaw_slot), math.sin(yaw_slot))
            goal.pose.position.x = center[0] - back * u_long[0]
            goal.pose.position.y = center[1] - back * u_long[1]
        goal.pose.position.z = 0.0
        # Stage-2 goal heading: match Stage-1 heading if available; otherwise fallback to slot yaw
        if self._last_stage1_goal is not None:
            goal.pose.orientation = self._last_stage1_goal.pose.orientation
        else:
            goal.pose.orientation = yaw_to_quaternion(yaw_slot)
        
        # 충돌 검사: Stage 2 골 위치에 충돌이 있으면 None 반환
        vehicle_radius = float(self.get_parameter('vehicle_width').value) / 2.0 + float(self.get_parameter('safety_margin').value)
        if self._min_distance_to_cones(goal.pose.position.x, goal.pose.position.y) < vehicle_radius:
            self.get_logger().warn(f"DEBUG: Stage 2 goal has collision at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
            return None
            
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
        straight_only = bool(self.get_parameter('stage2_straight_only').value)

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

        # 직선만 생성하는 옵션
        if straight_only:
            x0 = start_pose.pose.pose.position.x
            y0 = start_pose.pose.pose.position.y
            x2 = goal.pose.position.x
            y2 = goal.pose.position.y
            dx = x2 - x0
            dy = y2 - y0
            L_line = math.hypot(dx, dy)
            n_line = max(1, int(L_line / ds))
            for i in range(n_line + 1):
                t = i / max(1, n_line)
                px = x0 + t * dx
                py = y0 + t * dy
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = px
                pose.pose.position.y = py
                pose.pose.orientation = yaw_to_quaternion(yaw_g)
                path.poses.append(pose)

            eff_radius = 0.5 * vehicle_width + safety_margin
            for p in path.poses:
                if self._min_distance_to_cones(p.pose.position.x, p.pose.position.y) < eff_radius:
                    self.get_logger().warn('Stage-2 straight path may collide with cones.')
                    break
            return path

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


def main(args=None) -> None:
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


