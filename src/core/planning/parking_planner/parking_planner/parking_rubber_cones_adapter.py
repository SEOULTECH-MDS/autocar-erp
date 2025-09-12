#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from planning_msgs.msg import ObstacleArray, Obstacle

import tf2_ros
# 중요: 아래 임포트는 tf2가 geometry_msgs 타입(PoseStamped 등) 변환을 지원하도록 타입 등록을 수행합니다.
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import TransformException


def _quat_multiply(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rotate_point_by_quaternion(p: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    # Rotate vector p by quaternion q
    px, py, pz = p
    qx, qy, qz, qw = q
    # p' = q * (p,0) * q_conj
    q_p = (px, py, pz, 0.0)
    q_conj = (-qx, -qy, -qz, qw)
    qp = _quat_multiply((qx, qy, qz, qw), q_p)
    qpq = _quat_multiply(qp, q_conj)
    return (qpq[0], qpq[1], qpq[2])


def _is_non_white(r: float, g: float, b: float) -> Tuple[bool, str]:
    # Consider white if all channels are high
    is_white = (r > 0.8) and (g > 0.8) and (b > 0.8)
    return (not is_white), 'nonwhite'


class ParkingRubberConesAdapter(Node):
    def __init__(self) -> None:
        super().__init__('parking_rubber_cones_adapter')

        self.declare_parameter('input_markers', '/sensor_fusion/tracked_rubber_cones')
        self.declare_parameter('output_obstacles', '/parking/cones_obstacles')
        self.declare_parameter('output_markers', '/parking/cones_obstacles_markers')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('cone_radius_m', 0.15)
        self.declare_parameter('marker_alpha', 0.9)
        self.declare_parameter('tf_timeout_sec', 0.5)
        # Right side rule: True → keep y<0 as right, False → keep y>0 as right
        self.declare_parameter('right_is_negative_y', True)
        # Debug control: when true, publish markers and print counters
        self.declare_parameter('debug_enabled', False)

        qos = QoSProfile(depth=10)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.obstacles_pub = self.create_publisher(ObstacleArray, self.get_parameter('output_obstacles').value, qos)
        self.markers_pub = self.create_publisher(MarkerArray, self.get_parameter('output_markers').value, qos)

        # Subscriber
        self.subscription = self.create_subscription(
            MarkerArray,
            self.get_parameter('input_markers').value,
            self._marker_array_callback,
            qos,
        )

        self._next_id = 1
        self.get_logger().info('parking_rubber_cones_adapter started')

    def _marker_array_callback(self, msg: MarkerArray) -> None:
        target_frame = str(self.get_parameter('target_frame').value)
        cone_radius = float(self.get_parameter('cone_radius_m').value)
        marker_alpha = float(self.get_parameter('marker_alpha').value)
        tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        right_is_negative_y = bool(self.get_parameter('right_is_negative_y').value)

        obstacles = ObstacleArray()
        obstacles.header.frame_id = target_frame
        obstacles.header.stamp = self.get_clock().now().to_msg()

        debug_enabled = bool(self.get_parameter('debug_enabled').value)
        viz_array = MarkerArray() if debug_enabled else None

        # debug counters (only used when debug_enabled)
        total_markers = 0
        total_points = 0
        kept_color = 0
        kept_right = 0
        tf_ok = 0
        tf_fail = 0

        for m in msg.markers:
            if debug_enabled:
                total_markers += 1
            # Color filter (나중에 yolo에서 '콘'을 인식할때는 제거)
            keep_color, color_name = _is_non_white(m.color.r, m.color.g, m.color.b)
            if not keep_color:
                continue
            if debug_enabled:
                kept_color += 1

            # Collect candidate points in the marker's frame
            points_in_src: List[Tuple[float, float, float]] = []
            if len(m.points) > 0:
                q = (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)
                t = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
                for p in m.points:
                    rp = _rotate_point_by_quaternion((p.x, p.y, p.z), q)
                    points_in_src.append((rp[0] + t[0], rp[1] + t[1], rp[2] + t[2]))
            else:
                points_in_src.append((m.pose.position.x, m.pose.position.y, m.pose.position.z))

            # Right-side filter in sensor frame (REP-103 default: y<0 is right)
            if debug_enabled:
                total_points += len(points_in_src)
            if right_is_negative_y:
                points_in_src = [p for p in points_in_src if p[1] < 0.0]
            else:
                points_in_src = [p for p in points_in_src if p[1] > 0.0]
            if not points_in_src:
                continue
            if debug_enabled:
                kept_right += len(points_in_src)

            # Transform each candidate point to target frame
            for p in points_in_src:
                ps = PoseStamped()
                ps.header.frame_id = m.header.frame_id
                # 최신 TF 사용(미래 외삽 방지) 위해 stamp=0 사용
                ps.header.stamp = rclpy.time.Time().to_msg()
                ps.pose.position.x = p[0]
                ps.pose.position.y = p[1]
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                try:
                    ps_map: PoseStamped = self.tf_buffer.transform(
                        ps,
                        target_frame,
                        timeout=Duration(seconds=tf_timeout),
                    )
                    if debug_enabled:
                        tf_ok += 1
                except Exception as e:
                    if debug_enabled:
                        tf_fail += 1
                    self.get_logger().warn(f"TF transform failed ({ps.header.frame_id}->{target_frame}): {e}")
                    continue

                # Create obstacle
                obs = Obstacle()
                obs.id = self._next_id
                self._next_id += 1
                obs.type = 'cone'
                obs.center.x = float(ps_map.pose.position.x)
                obs.center.y = float(ps_map.pose.position.y)
                obs.center.z = 0.0
                obs.radius = float(cone_radius)
                obs.description = f'right;{color_name}'
                obstacles.obstacles.append(obs)

                # Visualization marker (cylinder)
                mk = Marker()
                mk.header.frame_id = target_frame
                mk.header.stamp = self.get_clock().now().to_msg()
                mk.ns = 'parking_cones'
                mk.id = obs.id
                mk.type = Marker.CYLINDER
                mk.action = Marker.ADD
                mk.pose.position.x = obs.center.x
                mk.pose.position.y = obs.center.y
                mk.pose.position.z = 0.0
                mk.pose.orientation.w = 1.0
                diameter = 2.0 * cone_radius
                mk.scale.x = diameter
                mk.scale.y = diameter
                mk.scale.z = diameter
                mk.color.r = 1.0
                mk.color.g = 0.5
                mk.color.b = 0.0
                mk.color.a = marker_alpha
                if debug_enabled:
                    viz_array.markers.append(mk)

        if debug_enabled:
            self.get_logger().info(
                f"markers={total_markers}, pts_in={total_points}, color_kept={kept_color}, right_kept={kept_right}, tf_ok={tf_ok}, tf_fail={tf_fail}, publish={len(obstacles.obstacles)}"
            )

        if obstacles.obstacles:
            self.obstacles_pub.publish(obstacles)
        if debug_enabled and viz_array and viz_array.markers:
            self.markers_pub.publish(viz_array)


def main() -> None:
    rclpy.init()
    node = ParkingRubberConesAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


