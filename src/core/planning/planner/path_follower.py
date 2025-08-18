#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ObstacleArray, Obstacle


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
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

    def distance_to_point(self, px: float, py: float) -> float:
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


class ObstacleModel:
    def __init__(self) -> None:
        self._segments: List[Segment] = []
        self._circles: List[Tuple[float, float, float]] = []  # (cx, cy, r)

    def update_from(self, msg: ObstacleArray) -> None:
        self._segments.clear()
        self._circles.clear()
        for obs in msg.obstacles:
            desc = obs.description or ''
            if desc.startswith('seg:'):
                try:
                    payload = desc.split(':', 1)[1]
                    x1, y1, x2, y2, width = [float(x) for x in payload.split(',')]
                    self._segments.append(Segment(x1=x1, y1=y1, x2=x2, y2=y2, width=width))
                except Exception:
                    continue
            else:
                # Assume circular obstacle if radius > 0
                r_eff = float(obs.radius) if hasattr(obs, 'radius') else 0.0
                if r_eff > 0.0:
                    self._circles.append((float(obs.center.x), float(obs.center.y), r_eff))

    def min_distance_to_any(self, x: float, y: float) -> float:
        dists: List[float] = []
        for seg in self._segments:
            dists.append(seg.distance_to_point(x, y))
        for cx, cy, r in self._circles:
            dists.append(max(0.0, math.hypot(x - cx, y - cy) - r))
        return min(dists) if dists else float('inf')

    def min_distance_to_segments(self, x: float, y: float) -> float:
        if not self._segments:
            return float('inf')
        return min(seg.distance_to_point(x, y) for seg in self._segments)

    def min_distance_to_circles(self, x: float, y: float) -> float:
        if not self._circles:
            return float('inf')
        return min(max(0.0, math.hypot(x - cx, y - cy) - r) for cx, cy, r in self._circles)


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('path_follower_node')

        # Parameters
        self.declare_parameter('follow_mode', 'auto')  # 'auto' or one of: stage1, stage2, stage3
        self.declare_parameter('prefer_order', 'stage1,stage2,stage3')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('wheelbase', 1.0)
        self.declare_parameter('v_forward', 0.5)
        self.declare_parameter('v_reverse', -0.6)
        self.declare_parameter('vehicle_width', 1.16)
        self.declare_parameter('vehicle_length', 2.02)
        self.declare_parameter('vehicle_height', 1.0)
        self.declare_parameter('safety_margin', 0.2)
        self.declare_parameter('max_curvature', 1.5)  # 1/R_min
        self.declare_parameter('assume_stage2_reverse', True)
        # Goal stopping behavior
        self.declare_parameter('stop_at_goal', True)
        self.declare_parameter('goal_pos_tol', 0.25)          # [m]
        self.declare_parameter('goal_yaw_tol_deg', 10.0)      # [deg]
        self.declare_parameter('goal_stop_time', 1.0)         # [s]
        # Early stop when heading ~ slot heading during stage2
        self.declare_parameter('early_stop_yaw_tol_deg', 8.0)
        # Visualization
        self.declare_parameter('show_vehicle_marker', True)
        # Collision horizon scaling
        self.declare_parameter('collision_horizon_scale', 1.5)

        # State
        self._path_stage1: Optional[Path] = None
        self._path_stage2: Optional[Path] = None
        self._path_stage3: Optional[Path] = None
        self._current_pose: Optional[PoseStamped] = None
        self._active_stage: Optional[str] = None
        self._eval_cache = {}

        # Obstacle model
        self._obstacles = ObstacleModel()

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._debug_pub = self.create_publisher(MarkerArray, '/follower_debug', 10)

        # Subscribers
        self.create_subscription(Path, '/stage1_path', self._on_stage1_path, 10)
        self.create_subscription(Path, '/stage2_path', self._on_stage2_path, 10)
        self.create_subscription(Path, '/stage3_path', self._on_stage3_path, 10)
        self.create_subscription(PoseStamped, '/current_pose', self._on_current_pose, 10)
        self.create_subscription(ObstacleArray, '/virtual_walls', self._on_virtual_walls, 10)
        self.create_subscription(PoseStamped, '/open_slot_pose', self._on_open_slot_pose, 10)

        # Timer
        period = 1.0 / max(1e-3, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._on_timer)

        self.get_logger().info('PathFollower node started.')
        # Internal goal hold timer (wall-clock seconds)
        self._goal_hold_until_sec: Optional[float] = None
        # Auto stage sequence state
        self._stage_sequence = ['stage1', 'stage2', 'stage3']
        self._stage_index = 0  # 0=stage1, 1=stage2, 2=stage3
        # Open slot pose cache
        self._open_slot_pose: Optional[PoseStamped] = None

    # ─────────────── Subscriptions ───────────────
    def _on_stage1_path(self, msg: Path) -> None:
        self._path_stage1 = msg
        self._eval_cache['stage1'] = self._evaluate_path(msg, 'stage1')

    def _on_stage2_path(self, msg: Path) -> None:
        self._path_stage2 = msg
        self._eval_cache['stage2'] = self._evaluate_path(msg, 'stage2')

    def _on_stage3_path(self, msg: Path) -> None:
        self._path_stage3 = msg
        self._eval_cache['stage3'] = self._evaluate_path(msg, 'stage3')

    def _on_current_pose(self, msg: PoseStamped) -> None:
        self._current_pose = msg

    def _on_virtual_walls(self, msg: ObstacleArray) -> None:
        self._obstacles.update_from(msg)

    def _on_open_slot_pose(self, msg: PoseStamped) -> None:
        self._open_slot_pose = msg

    # ─────────────── Core timer loop ───────────────
    def _on_timer(self) -> None:
        if self._current_pose is None:
            return

        path, stage = self._select_active_path()
        if path is None or not path.poses:
            return

        # Evaluate and visualize once per path update
        self._publish_debug_for(stage)

        # Pure pursuit tracking
        lookahead = float(self.get_parameter('lookahead').value)
        v_forward = float(self.get_parameter('v_forward').value)
        v_reverse = float(self.get_parameter('v_reverse').value)
        wheelbase = float(self.get_parameter('wheelbase').value)
        max_kappa = float(self.get_parameter('max_curvature').value)

        cx = self._current_pose.pose.position.x
        cy = self._current_pose.pose.position.y
        cyaw = quaternion_to_yaw(self._current_pose.pose.orientation)

        # Goal stop handling (hold for a short time after reaching)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._goal_hold_until_sec is not None:
            if now_sec < self._goal_hold_until_sec:
                self._publish_zero_vel()
                return
            else:
                self._goal_hold_until_sec = None

        # Early stop on yaw alignment with slot heading during stage2
        if stage == 'stage2' and self._open_slot_pose is not None:
            yaw_slot = quaternion_to_yaw(self._open_slot_pose.pose.orientation)
            yaw_tol = math.radians(float(self.get_parameter('early_stop_yaw_tol_deg').value))
            if abs(wrap_to_pi(yaw_slot - cyaw)) <= yaw_tol:
                hold = float(self.get_parameter('goal_stop_time').value)
                self._goal_hold_until_sec = now_sec + max(0.0, hold)
                self._publish_zero_vel()
                return

        # Find nearest index
        nearest_idx = self._nearest_index(path, cx, cy)
        target_idx = self._advance_to_lookahead(path, nearest_idx, cx, cy, lookahead)
        target = path.poses[target_idx]

        # If goal reached (near last pose and yaw aligned), stop
        if bool(self.get_parameter('stop_at_goal').value):
            if self._is_goal_reached(path, cx, cy, cyaw):
                hold = float(self.get_parameter('goal_stop_time').value)
                self._goal_hold_until_sec = now_sec + max(0.0, hold)
                # Advance to next stage if in auto mode
                if str(self.get_parameter('follow_mode').value).lower() == 'auto':
                    self._advance_stage()
                self._publish_zero_vel()
                return

        heading = math.atan2(target.pose.position.y - cy, target.pose.position.x - cx)
        alpha = wrap_to_pi(heading - cyaw)
        kappa_cmd = 2.0 * math.sin(alpha) / max(lookahead, 1e-3)
        kappa_cmd = max(min(kappa_cmd, max_kappa), -max_kappa)

        # Determine motion direction
        reverse = (stage == 'stage2') and bool(self.get_parameter('assume_stage2_reverse').value)
        v_cmd = v_reverse if reverse else v_forward
        w_cmd = v_cmd * kappa_cmd  # bicycle model approx with small angles

        # Collision guard on short horizon ahead (mixed model)
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        vehicle_length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        line_eff = 0.5 * vehicle_width + safety_margin
        circ_eff = 0.5 * math.hypot(vehicle_width, vehicle_length) + safety_margin
        horizon_scale = float(self.get_parameter('collision_horizon_scale').value)
        if self._imminent_collision(path, nearest_idx, lookahead * max(0.0, horizon_scale), line_eff, circ_eff):
            self.get_logger().warn('Imminent collision detected on path. Holding position.')
            self._publish_zero_vel()
            return

        # Publish cmd_vel
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        self._cmd_pub.publish(twist)

    # ─────────────── Goal check ───────────────
    def _is_goal_reached(self, path: Path, cx: float, cy: float, cyaw: float) -> bool:
        if not path.poses:
            return False
        goal = path.poses[-1]
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        gyaw = quaternion_to_yaw(goal.pose.orientation)
        pos_tol = float(self.get_parameter('goal_pos_tol').value)
        yaw_tol = math.radians(float(self.get_parameter('goal_yaw_tol_deg').value))
        d = math.hypot(cx - gx, cy - gy)
        dyaw = abs(wrap_to_pi(gyaw - cyaw))
        return (d <= pos_tol) and (dyaw <= yaw_tol)

    def _publish_zero_vel(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._cmd_pub.publish(twist)

    # ─────────────── Stage progression ───────────────
    def _advance_stage(self) -> None:
        if self._stage_index < len(self._stage_sequence) - 1:
            self._stage_index += 1
            self.get_logger().info(f'Advancing to next stage: {self._stage_sequence[self._stage_index]}')
        else:
            self.get_logger().info('All stages completed.')

    # ─────────────── Collision horizon check ───────────────
    def _imminent_collision(self, path: Path, start_idx: int, horizon_len: float, line_eff_radius: float, circ_eff_radius: float) -> bool:
        if not path.poses:
            return False
        accum = 0.0
        last_x = path.poses[start_idx].pose.position.x
        last_y = path.poses[start_idx].pose.position.y
        for i in range(start_idx, len(path.poses)):
            px = path.poses[i].pose.position.x
            py = path.poses[i].pose.position.y
            accum += math.hypot(px - last_x, py - last_y)
            last_x, last_y = px, py
            seg_d = self._obstacles.min_distance_to_segments(px, py)
            cir_d = self._obstacles.min_distance_to_circles(px, py)
            if (seg_d < line_eff_radius) or (cir_d < circ_eff_radius):
                return True
            if accum >= horizon_len:
                break
        return False

    # ─────────────── Path selection ───────────────
    def _select_active_path(self) -> Tuple[Optional[Path], Optional[str]]:
        mode = str(self.get_parameter('follow_mode').value).lower()
        if mode in ('stage1', 'stage2', 'stage3'):
            path = getattr(self, f'_path_{mode}')
            self._active_stage = mode if (path is not None and path.poses) else None
            return (path if self._active_stage else None, self._active_stage)

        # Auto: follow fixed sequence stage1->stage2->stage3
        sequence = self._stage_sequence
        # Clamp index to available stage that has a path
        for i in range(self._stage_index, len(sequence)):
            s = sequence[i]
            path = getattr(self, f'_path_{s}', None)
            if path is not None and path.poses:
                self._stage_index = i
                self._active_stage = s
                return path, s
        # Fallback: search any available in order
        for s in sequence:
            path = getattr(self, f'_path_{s}', None)
            if path is not None and path.poses:
                self._active_stage = s
                return path, s
        self._active_stage = None
        return None, None

    # ─────────────── Evaluation and debugging ───────────────
    def _evaluate_path(self, path: Path, stage: str) -> dict:
        ds_min = 1e-3
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        vehicle_length = float(self.get_parameter('vehicle_length').value) if self.has_parameter('vehicle_length') else 2.02
        safety_margin = float(self.get_parameter('safety_margin').value)
        line_eff = 0.5 * vehicle_width + safety_margin
        circ_eff = 0.5 * math.hypot(vehicle_width, vehicle_length) + safety_margin
        max_kappa = float(self.get_parameter('max_curvature').value)

        min_margin = float('inf')
        max_curv = 0.0

        last = None
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            # Clearance margins against segments and circles
            seg_d = self._obstacles.min_distance_to_segments(x, y)
            cir_d = self._obstacles.min_distance_to_circles(x, y)
            margin = min(seg_d - line_eff, cir_d - circ_eff)
            min_margin = min(min_margin, margin)
            # Curvature estimate
            if last is not None:
                dx = x - last[0]
                dy = y - last[1]
                ds = max(ds_min, math.hypot(dx, dy))
                yaw = quaternion_to_yaw(pose.pose.orientation)
                yaw_prev = last[2]
                kappa = abs(wrap_to_pi(yaw - yaw_prev)) / ds
                if math.isfinite(kappa):
                    max_curv = max(max_curv, kappa)
            last = (x, y, quaternion_to_yaw(pose.pose.orientation))

        collision_free = (min_margin >= 0.0)
        trackable = (max_curv <= max_kappa)
        return {
            'stage': stage,
            'collision_free': collision_free,
            'min_clearance_margin': float(min_margin if math.isfinite(min_margin) else -float('inf')),
            'max_curvature': float(max_curv),
            'trackable': trackable,
            'line_eff': float(line_eff),
            'circ_eff': float(circ_eff),
            'max_kappa': float(max_kappa),
        }

    def _publish_debug_for(self, stage: Optional[str]) -> None:
        if stage is None:
            return
        evald = self._eval_cache.get(stage)
        if not evald:
            return
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'path_follower_status'
        mk.id = 0
        mk.type = Marker.TEXT_VIEW_FACING
        mk.action = Marker.ADD
        mk.pose.position.x = 0.0
        mk.pose.position.y = 0.0
        mk.pose.position.z = 1.5
        mk.pose.orientation.w = 1.0
        mk.scale.z = 0.4
        mk.color.a = 1.0
        mk.color.r = 1.0 if (not evald['collision_free'] or not evald['trackable']) else 0.1
        mk.color.g = 0.1 if (not evald['collision_free'] or not evald['trackable']) else 1.0
        mk.color.b = 0.1
        mk.text = (
            f"{stage} | clear_margin>=0.00m: {'OK' if evald['collision_free'] else 'COLLIDE'}\n"
            f"margin={evald['min_clearance_margin']:.2f}m, line_eff={evald['line_eff']:.2f}, circ_eff={evald['circ_eff']:.2f}\n"
            f"kappa_max={evald['max_curvature']:.2f} (<= {evald['max_kappa']:.2f}): "
            f"{'OK' if evald['trackable'] else 'NOT TRACKABLE'}"
        )
        arr = MarkerArray()
        arr.markers.append(mk)
        if bool(self.get_parameter('show_vehicle_marker').value) and self._current_pose is not None:
            arr.markers.append(self._make_vehicle_marker())
        self._debug_pub.publish(arr)

    def _make_vehicle_marker(self) -> Marker:
        mk = Marker()
        frame_id = self._current_pose.header.frame_id if (self._current_pose and self._current_pose.header.frame_id) else 'map'
        mk.header.frame_id = frame_id
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'vehicle_box'
        mk.id = 1
        mk.type = Marker.CUBE
        mk.action = Marker.ADD

        length = float(self.get_parameter('vehicle_length').value)
        width = float(self.get_parameter('vehicle_width').value)
        height = float(self.get_parameter('vehicle_height').value)

        mk.pose = self._current_pose.pose
        mk.pose.position.z = (self._current_pose.pose.position.z if self._current_pose else 0.0) + 0.5 * height

        mk.scale.x = length
        mk.scale.y = width
        mk.scale.z = height

        mk.color.a = 0.5
        mk.color.r = 0.1
        mk.color.g = 0.9
        mk.color.b = 1.0

        return mk

    # ─────────────── Helpers ───────────────
    def _nearest_index(self, path: Path, x: float, y: float) -> int:
        best = 0
        best_d2 = float('inf')
        for i, pose in enumerate(path.poses):
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best = i
                best_d2 = d2
        return best

    def _advance_to_lookahead(self, path: Path, start_idx: int, x: float, y: float, Ld: float) -> int:
        accum = 0.0
        last_x = x
        last_y = y
        for i in range(start_idx, len(path.poses)):
            px = path.poses[i].pose.position.x
            py = path.poses[i].pose.position.y
            accum += math.hypot(px - last_x, py - last_y)
            if accum >= Ld:
                return i
            last_x, last_y = px, py
        return len(path.poses) - 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


