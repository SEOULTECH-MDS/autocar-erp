#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.logging import LoggingSeverity

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64, Float64, String

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

import numpy as np
import pyproj
from autocar_utils.euler_from_quaternion import euler_from_quaternion
import math

from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import threading
import time


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        # Ensure INFO logs are visible
        try:
            self.get_logger().set_level(LoggingSeverity.INFO)
        except Exception:
            pass

        # Parameters
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('map_origin.lat', 37.630117)
        self.declare_parameter('map_origin.lon', 127.081431)
        self.declare_parameter('lanelet2_map_path', '')
        self.declare_parameter('allowed_lanelet_subtypes', ['road'])
        self.declare_parameter('max_select_distance', 3.0)
        self.declare_parameter('stopline_ahead_margin', 0.2)
        self.declare_parameter('status_log_hz', 2.0)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        map_origin_lat = self.get_parameter('map_origin.lat').get_parameter_value().double_value
        map_origin_lon = self.get_parameter('map_origin.lon').get_parameter_value().double_value
        self.max_select_distance = self.get_parameter('max_select_distance').get_parameter_value().double_value
        self.allowed_lanelet_subtypes = list(self.get_parameter('allowed_lanelet_subtypes').get_parameter_value().string_array_value)
        self.stopline_ahead_margin = self.get_parameter('stopline_ahead_margin').get_parameter_value().double_value
        try:
            status_hz = float(self.get_parameter('status_log_hz').get_parameter_value().double_value)
        except Exception:
            status_hz = 2.0
        self._status_log_interval = 1.0 / status_hz if status_hz and status_hz > 0.0 else 1.0

        self.projector = UtmProjector(Origin(map_origin_lat, map_origin_lon))
        # Compute UTM origin (to switch between absolute/local frames)
        proj_string = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        utm_proj = pyproj.Proj(proj_string)
        self.map_origin_utm_x, self.map_origin_utm_y = utm_proj(map_origin_lon, map_origin_lat)

        # Load lanelet2 map
        self.lanelet_map = None
        self.allowed_lanelet_ids = set()
        self.stopline_linestrings = []
        self.lanelet_id_to_stoplines = {}
        map_path = self.get_parameter('lanelet2_map_path').get_parameter_value().string_value
        if not map_path:
            self.get_logger().error("Parameter 'lanelet2_map_path' is empty. Cannot load map.")
        else:
            try:
                self.lanelet_map = lanelet2.io.load(map_path, self.projector)
                self.get_logger().info(f"Lanelet2 map loaded: {map_path}")
                # Precompute allowed lanelet ids
                allowed_subtypes_lower = set(s.lower() for s in (self.allowed_lanelet_subtypes or []))
                for ll in self.lanelet_map.laneletLayer:
                    subtype_val = self._get_attr(ll.attributes, 'subtype').lower()
                    ped_val = self._get_attr(ll.attributes, 'participant:pedestrian').lower()
                    ltype_val = self._get_attr(ll.attributes, 'type').lower()
                    if allowed_subtypes_lower and subtype_val not in allowed_subtypes_lower:
                        continue
                    if ped_val == 'yes':
                        continue
                    if ltype_val == 'stop_line':
                        continue
                    self.allowed_lanelet_ids.add(ll.id)

                # Collect stop line LineStrings (type or subtype == 'stop_line')
                try:
                    for ls in self.lanelet_map.lineStringLayer:
                        try:
                            ltype = self._get_attr(ls.attributes, 'type').lower()
                            subtype = self._get_attr(ls.attributes, 'subtype').lower()
                            if (ltype in ('stop_line', 'stopline')) or (subtype in ('stop_line', 'stopline')):
                                self.stopline_linestrings.append(ls)
                        except Exception:
                            pass
                    self.get_logger().info(f"Found {len(self.stopline_linestrings)} stop line LineStrings (including 'stopline')")
                except Exception as e:
                    self.get_logger().warn(f"Failed to scan stop line LineStrings: {e}")

                # Heuristic: determine whether lanelet coords are absolute UTM (large values) or local
                self.lanelet_coords_are_absolute = False
                try:
                    any_ll = next(iter(self.lanelet_map.laneletLayer))
                    test_pt = any_ll.leftBound[0]
                    if np.hypot(test_pt.x, test_pt.y) > 1e5:
                        self.lanelet_coords_are_absolute = True
                    self.get_logger().info(
                        f"Lanelet coords detected as {'UTM-absolute' if self.lanelet_coords_are_absolute else 'map-local'}"
                    )
                except Exception:
                    self.lanelet_coords_are_absolute = False

                # Map stoplines to lanelets via attribute ref_lanelet_id (supports 'ref_lanlet_id' fallback)
                try:
                    for ls in self.stopline_linestrings:
                        ref_val = self._get_attr(ls.attributes, 'ref_lanelet_id')
                        if ref_val in (None, ''):
                            ref_val = self._get_attr(ls.attributes, 'ref_lanlet_id')
                        try:
                            ref_id = int(ref_val)
                        except Exception:
                            continue
                        self.lanelet_id_to_stoplines.setdefault(ref_id, []).append(ls)
                    self.get_logger().info(
                        f"Lanelet->stopline links (by ref): {len(self.lanelet_id_to_stoplines)} lanelets have stoplines")
                except Exception as e:
                    self.get_logger().warn(f"Failed to map stoplines by ref id: {e}")

            except Exception as e:
                self.get_logger().error(f"Failed to load lanelet2 map: {e}")

        # Publishers / Subscribers
        qos_default = QoSProfile(depth=10)
        self.current_lanelet_pub = self.create_publisher(Int64, '/current_lanelet_id', qos_default)
        # Projection distance (signed along vehicle heading) as the default/only topic
        self.stopline_proj_distance_pub = self.create_publisher(Float64, '/stopline_distance', qos_default)
        self.stopline_type_pub = self.create_publisher(String, '/stopline_type', qos_default)
        self.location_sub = self.create_subscription(Odometry, '/autocar/location', self._on_location, qos_default)

        # State
        self.last_lanelet_id = None
        self.last_stopline_proj_distance = None
        self.last_stopline_type = None

        # Periodic status print (1 Hz) using wall clock (independent of ROS time)
        self._status_stop_event = threading.Event()
        self._status_thread = threading.Thread(target=self._status_wall_loop, daemon=True)
        self._status_thread.start()
        # Periodic republish lanelet id (for latched/late subscribers downstream)
        self.repub_timer = self.create_timer(1.0, self._republish_lanelet)

        # TF buffer/listener for world->map coordinate transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _get_attr(self, attr_map, key: str) -> str:
        try:
            if hasattr(attr_map, 'get'):
                v = attr_map.get(key, '')
            else:
                try:
                    v = attr_map[key]
                except Exception:
                    v = ''
        except Exception:
            v = ''
        if v in (None, ''):
            try:
                for k in list(attr_map):
                    if str(k).lower() == key.lower():
                        try:
                            v = attr_map[k]
                        except Exception:
                            v = ''
                        break
            except Exception:
                pass
        return '' if v is None else str(v)

    def _on_location(self, odom: Odometry):
        if self.lanelet_map is None:
            return

        # Transform pose to 'map' frame if possible to match lanelet coordinates
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        try:
            target_frame = 'map'
            source_frame = odom.header.frame_id if odom.header.frame_id else 'world'
            if source_frame != target_frame:
                ps = PoseStamped()
                ps.header = odom.header
                ps.pose = odom.pose.pose
                tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                # Use PoseStamped-specific transform to avoid attribute errors
                ps_map = tf2_geometry_msgs.do_transform_pose_stamped(ps, tf)
                x = ps_map.pose.position.x
                y = ps_map.pose.position.y
        except Exception as e:
            # Fallback: use raw coordinates
            self.get_logger().warn(f"TF transform to 'map' failed, using raw: {e}")

        # Align point coordinates with lanelet coordinate frame
        if getattr(self, 'lanelet_coords_are_absolute', False):
            px = x + self.map_origin_utm_x
            py = y + self.map_origin_utm_y
        else:
            px = x
            py = y

        point_ll = lanelet2.core.BasicPoint2d(px, py)

        # Query nearest lanelets and pick the one that CONTAINS the point if possible
        try:
            k_neighbors = min(100, max(1, len(self.lanelet_map.laneletLayer)))
            nearest_list = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, point_ll, k_neighbors)
        except Exception as e:
            self.get_logger().warn(f"findNearest failed: {e}")
            return

        if not nearest_list:
            return

        chosen_id = None
        chosen_dist = None

        # 1) Prefer lanelet that contains the point
        for dist, cand_lanelet in nearest_list:
            if self.allowed_lanelet_ids and cand_lanelet.id not in self.allowed_lanelet_ids:
                continue
            try:
                if lanelet2.geometry.contains(cand_lanelet, point_ll):
                    chosen_id = cand_lanelet.id
                    chosen_dist = 0.0
                    break
            except Exception:
                pass

        # 2) If none contains, fall back to nearest allowed with distance check
        if chosen_id is None:
            for dist, cand_lanelet in nearest_list:
                if self.allowed_lanelet_ids and cand_lanelet.id not in self.allowed_lanelet_ids:
                    continue
                chosen_id = cand_lanelet.id
                chosen_dist = dist
                break

        if chosen_id is None:
            return

        if (chosen_dist is not None) and (chosen_dist > self.max_select_distance):
            # Too far away from any road lanelet
            return

        if chosen_id != self.last_lanelet_id:
            self.last_lanelet_id = chosen_id
            msg = Int64()
            msg.data = int(chosen_id)
            self.current_lanelet_pub.publish(msg)
            self.get_logger().info(f"현재 lanelet 업데이트 됨: {chosen_id}")
            # console print removed to avoid duplicate logs

        # Publish projection distance (signed along vehicle heading) to the nearest stop line
        try:
            # Vehicle forward unit vector from odometry yaw
            try:
                q = odom.pose.pose.orientation
                yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
                fwd = np.array([np.cos(yaw), np.sin(yaw)], dtype=float)
            except Exception:
                fwd = np.array([1.0, 0.0], dtype=float)

            # Candidate stoplines: only those explicitly linked to current lanelet by ref id
            if self.last_lanelet_id is not None and self.last_lanelet_id in self.lanelet_id_to_stoplines:
                candidates = self.lanelet_id_to_stoplines[self.last_lanelet_id]
            else:
                candidates = []

            # Choose the nearest stopline AHEAD of the vehicle along heading (signed projection >= margin)
            best_signed = None
            selected_ls = None
            for ls in candidates:
                d_ls, cp = self._closest_distance_and_point(px, py, ls)
                if d_ls is None or cp is None:
                    continue
                delta = np.array([cp[0] - px, cp[1] - py], dtype=float)
                signed = float(delta.dot(fwd))
                if signed >= self.stopline_ahead_margin:
                    if (best_signed is None) or (signed < best_signed):
                        best_signed = signed
                        selected_ls = ls

            # Publish projection distance (signed). If not available, publish NaN
            msg_proj = Float64()
            if best_signed is not None:
                msg_proj.data = float(best_signed)
                self.last_stopline_proj_distance = msg_proj.data
            else:
                msg_proj.data = math.nan
                self.last_stopline_proj_distance = None
            self.stopline_proj_distance_pub.publish(msg_proj)

            # Update last stopline type from selected stopline (if any)
            if selected_ls is not None:
                stype = self._get_attr(selected_ls.attributes, 'stopline_type')
                self.last_stopline_type = stype if stype else 'no_stopline'
            else:
                # No linked stopline ahead for this lanelet
                self.last_stopline_type = 'no_stopline'

            # Publish stopline type every cycle
            type_msg = String()
            type_msg.data = self.last_stopline_type if self.last_stopline_type else 'no_stopline'
            self.stopline_type_pub.publish(type_msg)
        except Exception as e:
            self.get_logger().warn(f"Stopline distance publish failed: {e}")

    def _closest_distance_and_point(self, px: float, py: float, ls):
        try:
            pts = list(ls)
            if len(pts) < 2:
                return None, None
            d_min_sq = None
            closest_pt = None
            for i in range(len(pts) - 1):
                x1, y1 = float(pts[i].x), float(pts[i].y)
                x2, y2 = float(pts[i+1].x), float(pts[i+1].y)
                # Project point onto segment
                vx, vy = x2 - x1, y2 - y1
                wx, wy = px - x1, py - y1
                seg_len_sq = vx * vx + vy * vy
                if seg_len_sq <= 1e-9:
                    t = 0.0
                else:
                    t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len_sq))
                proj_x = x1 + t * vx
                proj_y = y1 + t * vy
                dx = px - proj_x
                dy = py - proj_y
                dist_sq = dx * dx + dy * dy
                if (d_min_sq is None) or (dist_sq < d_min_sq):
                    d_min_sq = dist_sq
                    closest_pt = (proj_x, proj_y)
            return (float(np.sqrt(d_min_sq)) if d_min_sq is not None else None, closest_pt)
        except Exception:
            return None, None

    def _print_status(self):
        status = self.last_lanelet_id if self.last_lanelet_id is not None else 'None'
        dist = self.last_stopline_proj_distance if self.last_stopline_proj_distance is not None else 'NaN'
        stype = self.last_stopline_type if self.last_stopline_type is not None else 'no_stopline'
        self.get_logger().info(f"현재 Lanelet: {status}, 정지선 거리: {dist}, 정지선 타입: {stype}")
        # console print removed to avoid duplicate logs

    def _status_wall_loop(self):
        # Wall-clock based periodic logger, unaffected by use_sim_time or /clock
        while rclpy.ok() and not self._status_stop_event.is_set():
            try:
                self._print_status()
            except Exception as e:
                try:
                    self.get_logger().warn(f"Status print failed: {e}")
                except Exception:
                    pass
            time.sleep(self._status_log_interval)

    def shutdown(self):
        try:
            if hasattr(self, '_status_stop_event'):
                self._status_stop_event.set()
            # Give the thread a brief moment to exit
            if hasattr(self, '_status_thread') and self._status_thread.is_alive():
                self._status_thread.join(timeout=0.2)
        except Exception:
            pass

    def _republish_lanelet(self):
        if self.last_lanelet_id is not None:
            msg = Int64()
            msg.data = int(self.last_lanelet_id)
            self.current_lanelet_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


