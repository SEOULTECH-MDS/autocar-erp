#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ObstacleArray


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


@dataclass
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float
    width: float

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    @property
    def angle(self) -> float:
        return math.atan2(self.y2 - self.y1, self.x2 - self.x1)

    def distance_to_point(self, px: float, py: float) -> float:
        """Minimum distance from segment to point (px, py)."""
        vx = self.x2 - self.x1
        vy = self.y2 - self.y1
        wx = px - self.x1
        wy = py - self.y1
        seg_len2 = vx * vx + vy * vy
        if seg_len2 <= 1e-9:
            return math.hypot(px - self.x1, py - self.y1)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len2))
        proj_x = self.x1 + t * vx
        proj_y = self.y1 + t * vy
        return math.hypot(px - proj_x, py - proj_y)


class PlannerNode(Node):
    """Stage-1 parking planner.

    - Subscribes:
        /open_slot_pose (PoseStamped)
        /virtual_walls   (planning_msgs/ObstacleArray)
        /current_pose    (PoseStamped) [optional]
    - Publishes:
        /stage1_goal           (PoseStamped)
        /planner_debug_markers (MarkerArray)
    """

    def __init__(self) -> None:
        super().__init__('planner_node')

        # Parameters
        self.declare_parameter('yaw_offset_deg', 30.0)      # 좌측 선회 각도
        self.declare_parameter('s_overshoot', 0.6)          # (구버전) 슬롯 중심을 얼마나 지나칠지 [m]
        self.declare_parameter('front_margin', 0.5)         # 위쪽(전방) 경계선에서 추가 전방 여유 [m]
        self.declare_parameter('clear_lateral', 0.1)        # 도로쪽 여유 [m]
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
        self.declare_parameter('stage2_goal_offset_y', -1.0)  # map 프레임 y에서 -1m
        self.declare_parameter('stage2_back_offset', 1.0)     # 슬롯 진행축 기준 뒤로 1m (fallback)
        self.declare_parameter('stage2_straight_only', True)  # 원호 없이 직선 경로만 생성
        # Stage-2 guided reverse (no explicit goal pose)
        self.declare_parameter('stage2_guided', False)
        self.declare_parameter('stage2_inside_margin', 0.2)
        self.declare_parameter('stage2_k_lat', 0.8)
        self.declare_parameter('stage2_k_yaw', 0.8)
        self.declare_parameter('stage2_max_steps', 800)
        self.declare_parameter('stage2_lookahead', 1.0)
        # Stage-3
        self.declare_parameter('stage3_preview', True)

        # State
        self._open_slot_pose: Optional[PoseStamped] = None
        self._current_pose: Optional[PoseStamped] = None
        self._segments: List[Segment] = []
        self._lane_left_pts: List[Tuple[float, float]] = []
        self._lane_right_pts: List[Tuple[float, float]] = []
        self._last_stage1_goal: Optional[PoseStamped] = None

        # Publishers
        self._goal_pub = self.create_publisher(PoseStamped, '/stage1_goal', 10)
        self._stage2_goal_pub = self.create_publisher(PoseStamped, '/stage2_goal', 10)
        self._vis_pub = self.create_publisher(MarkerArray, '/planner_debug_markers', 10)
        self._path_pub = self.create_publisher(Path, '/stage1_path', 10)
        self._stage2_path_pub = self.create_publisher(Path, '/stage2_path', 10)
        self._stage3_path_pub = self.create_publisher(Path, '/stage3_path', 10)
        self._stage3_goal_pub = self.create_publisher(PoseStamped, '/stage3_goal', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/open_slot_pose', self._on_open_slot_pose, 10)
        self.create_subscription(ObstacleArray, '/virtual_walls', self._on_virtual_walls, 10)
        self.create_subscription(PoseStamped, '/current_pose', self._on_current_pose, 10)
        self.create_subscription(MarkerArray, '/road_markers', self._on_road_markers, 10)

        # Timer to re-evaluate goal periodically (in case only one topic updates)
        self.create_timer(0.2, self._maybe_publish_goal)

        self.get_logger().info('Planner node started (Stage-1 goal computation).')

    # ───────────────────────────── Callbacks ─────────────────────────────
    def _on_open_slot_pose(self, msg: PoseStamped) -> None:
        self._open_slot_pose = msg
        self._maybe_publish_goal()

    def _on_current_pose(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _on_virtual_walls(self, msg: ObstacleArray) -> None:
        self._segments = self._parse_segments(msg)
        self._maybe_publish_goal()

    def _on_road_markers(self, msg: MarkerArray) -> None:
        left: List[Tuple[float, float]] = []
        right: List[Tuple[float, float]] = []
        for mk in msg.markers:
            if mk.type != Marker.LINE_STRIP or not mk.points:
                continue
            ns = mk.ns.lower() if mk.ns else ''
            pts = [(float(p.x), float(p.y)) for p in mk.points]
            if 'lane_left' in ns:
                left.extend(pts)
            elif 'lane_right' in ns:
                right.extend(pts)
        self._lane_left_pts = left
        self._lane_right_pts = right

    # ───────────────────────────── Core Logic ────────────────────────────
    def _parse_segments(self, msg: ObstacleArray) -> List[Segment]:
        segments: List[Segment] = []
        for obs in msg.obstacles:
            desc = obs.description or ''
            if not desc.startswith('seg:'):
                continue
            try:
                # format: seg:x1,y1,x2,y2,width
                payload = desc.split(':', 1)[1]
                x1, y1, x2, y2, width = [float(x) for x in payload.split(',')]
                segments.append(Segment(x1=x1, y1=y1, x2=x2, y2=y2, width=width))
            except Exception:
                # Ignore malformed segments
                continue
        return segments

    def _estimate_slot_dimensions(self, yaw_slot: float, center: Tuple[float, float]) -> Tuple[float, float]:
        """Estimate (L, W) using virtual wall segments oriented w.r.t yaw_slot.

        Strategy: classify segments into longitudinal (parallel) and lateral (perpendicular)
        relative to yaw_slot, then take median lengths. Optionally focus on segments near the
        open slot center.
        """
        if not self._segments:
            return (
                float(self.get_parameter('default_slot_len').value),
                float(self.get_parameter('default_slot_width').value),
            )

        angle_eps = math.radians(float(self.get_parameter('angle_eps_deg').value))
        prefer_near = bool(self.get_parameter('prefer_near_open_slot').value)
        near_radius = float(self.get_parameter('near_radius').value)

        cx, cy = center
        long_lengths: List[float] = []
        lat_lengths: List[float] = []

        for seg in self._segments:
            if prefer_near:
                if seg.distance_to_point(cx, cy) > near_radius:
                    continue
            diff = abs(wrap_to_pi(seg.angle - yaw_slot))
            diff = min(diff, abs(math.pi - diff))  # account for 180° ambiguity
            if diff < angle_eps:
                long_lengths.append(seg.length)
            elif abs(diff - math.pi / 2.0) < angle_eps:
                lat_lengths.append(seg.length)

        def median(values: List[float]) -> Optional[float]:
            if not values:
                return None
            values_sorted = sorted(values)
            n = len(values_sorted)
            mid = n // 2
            if n % 2 == 1:
                return values_sorted[mid]
            return 0.5 * (values_sorted[mid - 1] + values_sorted[mid])

        L_est = median(long_lengths)
        W_est = median(lat_lengths)

        if L_est is None or W_est is None:
            # Fallback to defaults
            L_default = float(self.get_parameter('default_slot_len').value)
            W_default = float(self.get_parameter('default_slot_width').value)
            if L_est is None:
                L_est = L_default
            if W_est is None:
                W_est = W_default

        return float(L_est), float(W_est)

    def _compute_centerline_lateral(self, center: Tuple[float, float], u_lat: Tuple[float, float]) -> Optional[float]:
        """Compute lateral offset of road centerline from slot center along +u_lat.

        Returns distance d such that centerline_point ≈ center + d * u_lat. None if unavailable.
        """
        if not (self._lane_left_pts and self._lane_right_pts):
            return None
        def avg_xy(pts: List[Tuple[float, float]]) -> Tuple[float, float]:
            sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
            n = max(1, len(pts))
            return (sx / n, sy / n)
        lx, ly = avg_xy(self._lane_left_pts)
        rx, ry = avg_xy(self._lane_right_pts)
        cx, cy = center
        ux, uy = u_lat
        d_left = (lx - cx) * ux + (ly - cy) * uy
        d_right = (rx - cx) * ux + (ry - cy) * uy
        return 0.5 * (d_left + d_right)

    def _compute_stage1_goal(self, open_slot_pose: PoseStamped) -> Tuple[PoseStamped, MarkerArray]:
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        center = (open_slot_pose.pose.position.x, open_slot_pose.pose.position.y)

        # Dimensions from virtual walls (with default fallback)
        L, W = self._estimate_slot_dimensions(yaw_slot, center)

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

        # Lateral placement: prefer road centerline, then fallback to slot-based offset
        lateral = None
        if bool(self.get_parameter('prefer_centerline_lateral').value):
            lateral = self._compute_centerline_lateral(center, u_lat)
            if lateral is not None:
                lateral = lateral + float(self.get_parameter('centerline_left_margin').value)
        if lateral is None:
            lateral = (W / 2.0 + clear_lat)

        # Stop pose
        x_entry = center[0] + s_along * u_long[0] + lateral * u_lat[0]
        y_entry = center[1] + s_along * u_long[1] + lateral * u_lat[1]
        yaw_entry = wrap_to_pi(yaw_slot + yaw_offset)

        goal = PoseStamped()
        goal.header = open_slot_pose.header
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
        if self._open_slot_pose is None:
            return
        try:
            goal, markers = self._compute_stage1_goal(self._open_slot_pose)
        except Exception as e:
            self.get_logger().warn(f'Stage-1 goal computation failed: {e}')
            return
        self._goal_pub.publish(goal)
        self._vis_pub.publish(markers)
        # Optional: publish a straight+arc path from current pose
        if bool(self.get_parameter('show_stage1_path').value):
            path_msg = self._compute_stage1_path(self._current_pose, goal)
            if path_msg is not None:
                self._path_pub.publish(path_msg)
        # Compute and publish stage-2 goal/path as well
        self._last_stage1_goal = goal
        if bool(self.get_parameter('stage2_guided').value):
            stage2_path = self._compute_stage2_path_guided(self._last_stage1_goal, self._open_slot_pose)
            if stage2_path is not None:
                self._stage2_path_pub.publish(stage2_path)
        else:
            stage2_goal = self._compute_stage2_goal(self._open_slot_pose)
            if stage2_goal is not None:
                self._stage2_goal_pub.publish(stage2_goal)
                stage2_path = self._compute_stage2_path(self._last_stage1_goal, stage2_goal)
                if stage2_path is not None:
                    self._stage2_path_pub.publish(stage2_path)
                # Stage-3: define goal at slot center and plan from Stage-2 goal
                stage3_goal = self._compute_stage3_goal(self._open_slot_pose)
                if stage3_goal is not None:
                    self._stage3_goal_pub.publish(stage3_goal)
                    stage3_path = self._compute_stage3_path_from_stage2(stage2_goal, self._open_slot_pose)
                    if stage3_path is not None and stage3_path.poses:
                        self._stage3_path_pub.publish(stage3_path)

        # Stage-3 publish when inside slot and yaw mismatch (preview enabled)
        if bool(self.get_parameter('stage3_preview').value) and self._current_pose is not None and self._open_slot_pose is not None:
            yaw_slot = quaternion_to_yaw(self._open_slot_pose.pose.orientation)
            center = (self._open_slot_pose.pose.position.x, self._open_slot_pose.pose.position.y)
            L, W = self._estimate_slot_dimensions(yaw_slot, center)
            margin = float(self.get_parameter('stage2_inside_margin').value)
            if self._inside_slot(self._current_pose.pose.position.x,
                                 self._current_pose.pose.position.y,
                                 center, yaw_slot, L, W, margin):
                path3 = self._compute_stage3_path(self._current_pose, yaw_slot)
                if path3 is not None and path3.poses:
                    self._stage3_path_pub.publish(path3)

    # ───────────────────────────── Utilities ─────────────────────────────
    def _min_distance_to_walls(self, x: float, y: float) -> float:
        if not self._segments:
            return float('inf')
        return min(seg.distance_to_point(x, y) for seg in self._segments)

    def _compute_stage1_path(self, current_pose: Optional[PoseStamped], goal: PoseStamped) -> Optional[Path]:
        if current_pose is None:
            return None
        # Parameters
        R = float(self.get_parameter('turn_radius').value)
        yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        ds = float(self.get_parameter('path_resolution').value)
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        safety_margin = float(self.get_parameter('safety_margin').value)

        # Goal geometry
        yaw_g = quaternion_to_yaw(goal.pose.orientation)
        u_left = (-math.sin(yaw_g), math.cos(yaw_g))
        Cx = goal.pose.position.x + R * u_left[0]
        Cy = goal.pose.position.y + R * u_left[1]
        yaw_s = yaw_g - yaw_offset
        u_left_s = (-math.sin(yaw_s), math.cos(yaw_s))
        Psx = Cx - R * u_left_s[0]
        Psy = Cy - R * u_left_s[1]

        # Path message
        path = Path()
        path.header = goal.header

        # 1) Straight from current -> Ps
        x0 = current_pose.pose.position.x
        y0 = current_pose.pose.position.y
        dx = Psx - x0
        dy = Psy - y0
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
            pose.pose.orientation = yaw_to_quaternion(math.atan2(dy, dx) if L_line > 1e-6 else yaw_s)
            path.poses.append(pose)

        # 2) Left arc from yaw_s -> yaw_g, center C, radius R
        arc_len = abs(yaw_offset) * R
        n_arc = max(1, int(arc_len / ds))
        for j in range(1, n_arc + 1):
            th = yaw_s + (yaw_offset) * (j / n_arc)
            n_left = (-math.sin(th), math.cos(th))
            px = Cx - R * n_left[0]
            py = Cy - R * n_left[1]
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation = yaw_to_quaternion(th)
            path.poses.append(pose)

        # 3) Simple collision audit against virtual walls
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin
        collided = False
        for p in path.poses:
            d = self._min_distance_to_walls(p.pose.position.x, p.pose.position.y)
            if d < eff_radius:
                collided = True
                break
        if collided:
            self.get_logger().warn('Stage-1 path may collide with virtual walls. Consider increasing front_margin/clear_lateral/turn_radius.')

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

    def _compute_stage2_path_guided(self, start_pose: Optional[PoseStamped], open_slot_pose: Optional[PoseStamped]) -> Optional[Path]:
        if start_pose is None or open_slot_pose is None:
            return None
        # Slot geometry
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        center = (open_slot_pose.pose.position.x, open_slot_pose.pose.position.y)
        L, W = self._estimate_slot_dimensions(yaw_slot, center)

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

        # Initialize
        x = start_pose.pose.position.x
        y = start_pose.pose.position.y
        yaw = quaternion_to_yaw(start_pose.pose.orientation)
        path = Path(); path.header = open_slot_pose.header

        u_long = (math.cos(yaw_slot), math.sin(yaw_slot))
        # Target point: 슬롯 뒤쪽 내부 한 점 (센터에서 -L/2+margin)
        target_x = - (L / 2.0 - margin)
        target_world = (
            center[0] + target_x * u_long[0],
            center[1] + target_x * u_long[1],
        )

        # Guidance loop: reverse pure-pursuit towards target point
        for step in range(max_steps):
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)

            if self._inside_slot(x, y, center, yaw_slot, L, W, margin):
                break

            # Heading to target
            dx = target_world[0] - x
            dy = target_world[1] - y
            heading_to_target = math.atan2(dy, dx)
            # Pure pursuit curvature (reverse: 동일 공식, 통합에서 후진 적용)
            alpha = wrap_to_pi(heading_to_target - yaw)
            kappa_cmd = 2.0 * math.sin(alpha) / max(Ld, 1e-3)
            # Blend with error-based term for stability near axis
            x_l, y_l = self._to_slot_frame(x, y, center, yaw_slot)
            e_lat = y_l
            e_yaw = wrap_to_pi(yaw_slot - yaw)
            kappa_cmd += k_lat * e_lat + k_yaw * e_yaw
            kappa_cmd = max(min(kappa_cmd, 1.0 / max(1e-3, R)), -1.0 / max(1e-3, R))

            # Integrate one backward step
            yaw_new = wrap_to_pi(yaw - kappa_cmd * ds)  # backward: heading change sign flipped
            x_new = x - ds * math.cos(yaw)
            y_new = y - ds * math.sin(yaw)

            # Collision guard
            width = float(self.get_parameter('vehicle_width').value)
            length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
            eff_radius = 0.5 * math.hypot(width, length) + safety_margin
            if self._min_distance_to_walls(x_new, y_new) < eff_radius:
                self.get_logger().warn('Stage-2 guided step would collide with virtual wall. Stopping early.')
                break

            x, y, yaw = x_new, y_new, yaw_new

        return path if path.poses else None

    # ───────────────────────────── Stage-3 (align heading) ───────────────
    def _compute_stage3_path(self, start_pose: PoseStamped, yaw_slot: float) -> Optional[Path]:
        yaw_s = quaternion_to_yaw(start_pose.pose.orientation)
        dyaw = wrap_to_pi(yaw_slot - yaw_s)
        tol = math.radians(float(self.get_parameter('stage3_yaw_tol_deg').value)) if self.has_parameter('stage3_yaw_tol_deg') else math.radians(5.0)
        ds = float(self.get_parameter('path_resolution').value)
        R = float(self.get_parameter('stage3_turn_radius').value) if self.has_parameter('stage3_turn_radius') else 2.0
        forward_extra = float(self.get_parameter('stage3_forward').value) if self.has_parameter('stage3_forward') else 0.4

        path = Path(); path.header = start_pose.header

        # Slot geometry and safety for in-slot and collision guard
        center = None
        L = None
        W = None
        if self._open_slot_pose is not None:
            center = (self._open_slot_pose.pose.position.x, self._open_slot_pose.pose.position.y)
            L, W = self._estimate_slot_dimensions(yaw_slot, center)
        margin = float(self.get_parameter('stage2_inside_margin').value) if self.has_parameter('stage2_inside_margin') else 0.2
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin

        if abs(dyaw) <= tol:
            x0 = start_pose.pose.position.x
            y0 = start_pose.pose.position.y
            n_steps = max(1, int(forward_extra / max(ds, 1e-3)))
            for i in range(n_steps + 1):
                s = (forward_extra * i) / max(1, n_steps)
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position.x = x0 + s * math.cos(yaw_s)
                pose.pose.position.y = y0 + s * math.sin(yaw_s)
                pose.pose.orientation = yaw_to_quaternion(yaw_s)
                # Guards: stay inside slot and avoid virtual walls
                if center is not None and L is not None and W is not None:
                    if not self._inside_slot(pose.pose.position.x, pose.pose.position.y, center, yaw_slot, L, W, margin):
                        break
                if self._min_distance_to_walls(pose.pose.position.x, pose.pose.position.y) < eff_radius:
                    self.get_logger().warn('Stage-3 forward preview would collide with virtual wall. Truncating path.')
                    break
                path.poses.append(pose)
            return path

        turn_left = dyaw > 0.0
        n_left_s = (-math.sin(yaw_s), math.cos(yaw_s))
        Sx = start_pose.pose.position.x
        Sy = start_pose.pose.position.y
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
            # Guards: stay inside slot and avoid virtual walls
            if center is not None and L is not None and W is not None:
                if not self._inside_slot(px, py, center, yaw_slot, L, W, margin):
                    self.get_logger().warn('Stage-3 arc would exit slot bounds. Truncating path.')
                    break
            if self._min_distance_to_walls(px, py) < eff_radius:
                self.get_logger().warn('Stage-3 arc would collide with virtual wall. Truncating path.')
                break
            path.poses.append(pose)

        x_last = path.poses[-1].pose.position.x
        y_last = path.poses[-1].pose.position.y
        yaw_last = yaw_slot
        n_steps2 = max(1, int(forward_extra / max(ds, 1e-3)))
        for i in range(1, n_steps2 + 1):
            s = (forward_extra * i) / max(1, n_steps2)
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = x_last + s * math.cos(yaw_last)
            pose.pose.position.y = y_last + s * math.sin(yaw_last)
            pose.pose.orientation = yaw_to_quaternion(yaw_last)
            # Guards: stay inside slot and avoid virtual walls
            if center is not None and L is not None and W is not None:
                if not self._inside_slot(pose.pose.position.x, pose.pose.position.y, center, yaw_slot, L, W, margin):
                    break
            if self._min_distance_to_walls(pose.pose.position.x, pose.pose.position.y) < eff_radius:
                self.get_logger().warn('Stage-3 forward extension would collide with virtual wall. Truncating path.')
                break
            path.poses.append(pose)

        return path

    # ───────────────────────────── Stage-2 (reverse) ─────────────────────
    def _compute_stage2_goal(self, open_slot_pose: Optional[PoseStamped]) -> Optional[PoseStamped]:
        if open_slot_pose is None:
            return None
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        center = (open_slot_pose.pose.position.x, open_slot_pose.pose.position.y)
        goal = PoseStamped()
        goal.header = open_slot_pose.header
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
        return goal

    def _compute_stage3_goal(self, open_slot_pose: Optional[PoseStamped]) -> Optional[PoseStamped]:
        if open_slot_pose is None:
            return None
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        goal = PoseStamped()
        goal.header = open_slot_pose.header
        goal.pose.position.x = open_slot_pose.pose.position.x
        goal.pose.position.y = open_slot_pose.pose.position.y
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw_slot)
        return goal

    def _compute_stage3_path_from_stage2(self, start_pose: PoseStamped, open_slot_pose: PoseStamped) -> Optional[Path]:
        yaw_slot = quaternion_to_yaw(open_slot_pose.pose.orientation)
        center = (open_slot_pose.pose.position.x, open_slot_pose.pose.position.y)
        ds = float(self.get_parameter('path_resolution').value)
        R = float(self.get_parameter('stage3_turn_radius').value) if self.has_parameter('stage3_turn_radius') else 2.0
        width = float(self.get_parameter('vehicle_width').value)
        length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        eff_radius = 0.5 * math.hypot(width, length) + safety_margin
        L, W = self._estimate_slot_dimensions(yaw_slot, center)
        margin = float(self.get_parameter('stage2_inside_margin').value) if self.has_parameter('stage2_inside_margin') else 0.2

        # 1) Align yaw from start to yaw_slot via arc
        yaw_s = quaternion_to_yaw(start_pose.pose.orientation)
        dyaw = wrap_to_pi(yaw_slot - yaw_s)
        path = Path(); path.header = open_slot_pose.header
        turn_left = dyaw > 0.0
        n_left_s = (-math.sin(yaw_s), math.cos(yaw_s))
        Sx = start_pose.pose.position.x
        Sy = start_pose.pose.position.y
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
            if self._min_distance_to_walls(px, py) < eff_radius:
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
            if self._min_distance_to_walls(px, py) < eff_radius:
                break
            path.poses.append(pose)

        return path if path.poses else None

    def _compute_stage2_path(self, start_pose: Optional[PoseStamped], goal: PoseStamped) -> Optional[Path]:
        if start_pose is None:
            return None
        # Parameters
        R = float(self.get_parameter('turn_radius').value)
        ds = float(self.get_parameter('path_resolution').value)
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        safety_margin = float(self.get_parameter('safety_margin').value)
        straight_only = bool(self.get_parameter('stage2_straight_only').value)

        yaw_s = quaternion_to_yaw(start_pose.pose.orientation)
        yaw_g = quaternion_to_yaw(goal.pose.orientation)
        # 원하는 것은 시작(yaw_s = yaw_slot+30°)에서 목표(yaw_g=yaw_slot)로 30° 만큼 우회전하며 후진
        yaw_delta = wrap_to_pi(yaw_g - yaw_s)
        # 우회전 필요(음수). 양수면 좌회전으로 처리
        turn_right = yaw_delta < 0.0
        alpha = abs(yaw_delta)

        # 회전 중심: 시작점에서 법선 방향으로 R만큼 (우측: -n_left, 좌측: +n_left)
        n_left_s = (-math.sin(yaw_s), math.cos(yaw_s))
        Sx = start_pose.pose.position.x
        Sy = start_pose.pose.position.y
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
            x0 = start_pose.pose.position.x
            y0 = start_pose.pose.position.y
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
                if self._min_distance_to_walls(p.pose.position.x, p.pose.position.y) < eff_radius:
                    self.get_logger().warn('Stage-2 straight path may collide with virtual walls.')
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
            if self._min_distance_to_walls(p.pose.position.x, p.pose.position.y) < eff_radius:
                self.get_logger().warn('Stage-2 path may collide with virtual walls. Tune clearances/turn_radius.')
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


