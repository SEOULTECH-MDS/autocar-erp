#!/usr/bin/env python3
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional, List, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from planning_msgs.msg import ObstacleArray, Obstacle

import tf2_ros
import math


@dataclass
class ConeTrack:
    track_id: int
    x: float
    y: float
    radius: float
    description: str = ""
    count: int = 1
    last_seen_sec: float = 0.0


class ParkingConesMapper(Node):
    def __init__(self) -> None:
        super().__init__('parking_cones_mapper')

        # Parameters
        self.declare_parameter('input_topic', '/parking/cones_obstacles')
        self.declare_parameter('output_topic', '/parking/cones_mapped')
        self.declare_parameter('markers_topic', '/parking/cones_mapped_markers')
        self.declare_parameter('merge_distance_m', 0.5)
        self.declare_parameter('ema_alpha', 0.3)  # 0<alpha<=1, higher = more reactive
        self.declare_parameter('forget_time_sec', 2.0)
        self.declare_parameter('min_count_to_publish', 2)
        self.declare_parameter('max_tracks', 200)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('marker_alpha', 0.9)
        # ROI/dwell publishing (map frame output)
        self.declare_parameter('roi_y_min_m', -3.0)
        self.declare_parameter('roi_y_max_m', 0.0)
        self.declare_parameter('roi_dwell_time_sec', 0.3)
        self.declare_parameter('roi_output_topic', '/parking/cones_mapped_in_roi')
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('promoted_markers_topic', '/parking/cones_promoted_markers')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=20,
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        markers_topic = self.get_parameter('markers_topic').value

        self.subscription = self.create_subscription(
            ObstacleArray,
            input_topic,
            self._on_obstacles,
            qos,
        )
        self.pub_obstacles = self.create_publisher(ObstacleArray, output_topic, qos)
        self.pub_markers = self.create_publisher(MarkerArray, markers_topic, qos)
        self.pub_roi_obstacles = self.create_publisher(ObstacleArray, str(self.get_parameter('roi_output_topic').value), qos)
        self.pub_promoted_markers = self.create_publisher(MarkerArray, str(self.get_parameter('promoted_markers_topic').value), qos)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._publish_state)

        # Tracks state
        self._next_id: int = 1
        self._tracks: Dict[int, ConeTrack] = {}
        self._frame_id: str = 'map'
        # ROI dwell state by track id (first time in ROI, seconds)
        self._roi_since: Dict[int, Optional[float]] = {}
        # ROI promoted tracks: once dwell satisfied, keep until track forgotten
        self._roi_promoted: Set[int] = set()
        # Promotion time per track id
        self._promoted_since: Dict[int, float] = {}

        # TF for ROI test (map -> base_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('parking_cones_mapper started')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _find_best_match(self, x: float, y: float, merge_dist: float) -> Optional[int]:
        best_id: Optional[int] = None
        best_d2: float = merge_dist * merge_dist
        for tid, t in self._tracks.items():
            dx = t.x - x
            dy = t.y - y
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best_d2 = d2
                best_id = tid
        return best_id

    def _update_or_create_track(self, x: float, y: float, radius: float, desc: str) -> None:
        merge_dist = float(self.get_parameter('merge_distance_m').value)
        alpha = float(self.get_parameter('ema_alpha').value)
        t_id = self._find_best_match(x, y, merge_dist)
        now = self._now_sec()
        if t_id is None:
            if len(self._tracks) >= int(self.get_parameter('max_tracks').value):
                # Drop oldest
                oldest_id = min(self._tracks, key=lambda k: self._tracks[k].last_seen_sec)
                del self._tracks[oldest_id]
            t_id = self._next_id
            self._next_id += 1
            self._tracks[t_id] = ConeTrack(track_id=t_id, x=x, y=y, radius=radius, description=desc, count=1, last_seen_sec=now)
        else:
            t = self._tracks[t_id]
            t.x = (1.0 - alpha) * t.x + alpha * x
            t.y = (1.0 - alpha) * t.y + alpha * y
            t.radius = (1.0 - alpha) * t.radius + alpha * radius
            t.count += 1
            # Keep the first non-empty description if exists
            if not t.description and desc:
                t.description = desc
            t.last_seen_sec = now

    def _prune_tracks(self) -> None:
        forget = float(self.get_parameter('forget_time_sec').value)
        now = self._now_sec()
        to_delete: List[int] = []
        for tid, t in self._tracks.items():
            if (now - t.last_seen_sec) > forget:
                to_delete.append(tid)
        for tid in to_delete:
            del self._tracks[tid]
            if tid in self._roi_since:
                del self._roi_since[tid]
            if tid in self._roi_promoted:
                self._roi_promoted.discard(tid)
            if tid in self._promoted_since:
                del self._promoted_since[tid]

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        # Assume all in map frame already
        self._frame_id = msg.header.frame_id or 'map'

        for obs in msg.obstacles:
            x = float(obs.center.x)
            y = float(obs.center.y)
            radius = float(obs.radius) if obs.radius > 0.0 else 0.15
            desc = str(obs.description) if obs.description else ''
            self._update_or_create_track(x, y, radius, desc)

        self._prune_tracks()

    def _publish_state(self) -> None:
        min_count = int(self.get_parameter('min_count_to_publish').value)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id

        out = ObstacleArray()
        out.header = header
        markers = MarkerArray()
        promoted_markers = MarkerArray()

        # Prepare ROI transform and thresholds
        tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        y_min = float(self.get_parameter('roi_y_min_m').value)
        y_max = float(self.get_parameter('roi_y_max_m').value)
        dwell = float(self.get_parameter('roi_dwell_time_sec').value)
        try:
            # base_link <- map
            tf_bl_map = self.tf_buffer.lookup_transform('base_link', self._frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=tf_timeout))
            tx = float(tf_bl_map.transform.translation.x)
            ty = float(tf_bl_map.transform.translation.y)
            qx = float(tf_bl_map.transform.rotation.x)
            qy = float(tf_bl_map.transform.rotation.y)
            qz = float(tf_bl_map.transform.rotation.z)
            qw = float(tf_bl_map.transform.rotation.w)
            s2 = 2.0 * (qw * qz + qx * qy)
            c2 = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(s2, c2)
            c = math.cos(yaw)
            s_ = math.sin(yaw)
            tf_ok = True
        except Exception:
            tf_ok = False

        roi_out = ObstacleArray()
        roi_out.header = header

        for t in self._tracks.values():
            if t.count < min_count:
                continue
            obs = Obstacle()
            obs.id = t.track_id
            obs.type = 'cone_mapped'
            obs.center.x = t.x
            obs.center.y = t.y
            obs.center.z = 0.0
            obs.radius = t.radius
            obs.description = f'mapped;count={t.count}' if not t.description else f'{t.description};mapped;count={t.count}'
            out.obstacles.append(obs)

            # ROI dwell evaluation (map -> base_link to test y), promote once then persist
            tid = t.track_id
            promoted = (tid in self._roi_promoted)
            since = self._roi_since.get(tid)
            now = self._now_sec()
            if tf_ok:
                y_bl = s_ * t.x + c * t.y + ty
                in_roi_now = (y_bl >= y_min and y_bl <= y_max)
                if not promoted:
                    if in_roi_now:
                        if since is None:
                            self._roi_since[tid] = now
                            since = now
                        if since is not None and (now - since) >= dwell:
                            self._roi_promoted.add(tid)
                            self._promoted_since[tid] = now
                            promoted = True
                    else:
                        if since is not None:
                            self._roi_since[tid] = None
                            since = None
            # Publish promoted tracks regardless of current TF/ROI status
            if (tid in self._roi_promoted):
                prom = Obstacle()
                prom.id = obs.id
                prom.type = obs.type
                prom.center = obs.center
                prom.radius = obs.radius
                roi_s = (self._roi_since.get(tid) or 0.0)
                prom_s = (self._promoted_since.get(tid) or now)
                base_desc = obs.description or ''
                extra = f"roi_since={roi_s:.3f};promoted_since={prom_s:.3f};promoted=1"
                prom.description = (base_desc + ';' + extra) if base_desc else extra
                roi_out.obstacles.append(prom)

            mk = Marker()
            mk.header = header
            mk.ns = 'parking_cones_mapped'
            mk.id = t.track_id
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position.x = t.x
            mk.pose.position.y = t.y
            mk.pose.position.z = 0.0
            mk.pose.orientation.w = 1.0
            d = 2.0 * t.radius
            mk.scale.x = d
            mk.scale.y = d
            mk.scale.z = d
            # orange color
            mk.color.r = 1.0
            mk.color.g = 0.5
            mk.color.b = 0.0
            mk.color.a = float(self.get_parameter('marker_alpha').value)
            markers.markers.append(mk)

            # ID text marker (map frame)
            txt = Marker()
            txt.header = header
            txt.ns = 'parking_cone_id'
            txt.id = 500000 + t.track_id
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = t.x
            txt.pose.position.y = t.y
            txt.pose.position.z = 0.6
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 0.95
            txt.text = str(t.track_id)
            markers.markers.append(txt)

            # promoted markers (map frame), distinct color
            if t.track_id in self._roi_promoted:
                pm = Marker()
                pm.header = header
                pm.ns = 'parking_cones_promoted'
                pm.id = 100000 + t.track_id
                pm.type = Marker.CYLINDER
                pm.action = Marker.ADD
                pm.pose.position.x = t.x
                pm.pose.position.y = t.y
                pm.pose.position.z = 0.0
                pm.pose.orientation.w = 1.0
                pm.scale.x = d*1.1
                pm.scale.y = d*1.1
                pm.scale.z = d*1.1
                # blue-ish to distinguish
                pm.color.r = 0.0
                pm.color.g = 1.0
                pm.color.b = 0.0
                pm.color.a = float(self.get_parameter('marker_alpha').value)
                promoted_markers.markers.append(pm)

        if out.obstacles:
            self.pub_obstacles.publish(out)
        if markers.markers:
            self.pub_markers.publish(markers)
        if roi_out.obstacles:
            self.pub_roi_obstacles.publish(roi_out)
        if promoted_markers.markers:
            self.pub_promoted_markers.publish(promoted_markers)


def main() -> None:
    rclpy.init()
    node = ParkingConesMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


