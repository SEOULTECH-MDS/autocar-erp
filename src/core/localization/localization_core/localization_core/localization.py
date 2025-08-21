#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.logging import LoggingSeverity

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

import numpy as np
import pyproj

from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs


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

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        map_origin_lat = self.get_parameter('map_origin.lat').get_parameter_value().double_value
        map_origin_lon = self.get_parameter('map_origin.lon').get_parameter_value().double_value
        self.max_select_distance = self.get_parameter('max_select_distance').get_parameter_value().double_value
        self.allowed_lanelet_subtypes = list(self.get_parameter('allowed_lanelet_subtypes').get_parameter_value().string_array_value)

        self.projector = UtmProjector(Origin(map_origin_lat, map_origin_lon))
        # Compute UTM origin (to switch between absolute/local frames)
        proj_string = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        utm_proj = pyproj.Proj(proj_string)
        self.map_origin_utm_x, self.map_origin_utm_y = utm_proj(map_origin_lon, map_origin_lat)

        # Load lanelet2 map
        self.lanelet_map = None
        self.allowed_lanelet_ids = set()
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
            except Exception as e:
                self.get_logger().error(f"Failed to load lanelet2 map: {e}")

        # Publishers / Subscribers
        qos_default = QoSProfile(depth=10)
        self.current_lanelet_pub = self.create_publisher(Int64, '/current_lanelet_id', qos_default)
        self.location_sub = self.create_subscription(Odometry, '/autocar/location', self._on_location, qos_default)

        # State
        self.last_lanelet_id = None

        # Periodic status print (1 Hz)
        self.status_timer = self.create_timer(1.0, self._print_status)

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
            self.get_logger().info(f"Current lanelet id: {chosen_id}")
            try:
                print(f"Current lanelet id: {chosen_id}", flush=True)
            except Exception:
                pass

    def _print_status(self):
        status = self.last_lanelet_id if self.last_lanelet_id is not None else 'None'
        self.get_logger().info(f"[Periodic] Current lanelet id: {status}")
        try:
            print(f"[Periodic] Current lanelet id: {status}", flush=True)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


