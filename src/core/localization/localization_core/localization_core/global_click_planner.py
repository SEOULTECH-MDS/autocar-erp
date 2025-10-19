#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import signal

import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QGraphicsView, QGraphicsScene, QLabel, QHBoxLayout)
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QPainterPath, QPen, QColor, QPainter
from copy import deepcopy

from geometry_msgs.msg import PoseArray, Pose, Point, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from autoware_map_msgs.msg import LaneletMapBin

from autocar_utils.yaw_to_quaternion import yaw_to_quaternion
from autocar_utils.utils import CubicSpline2D
# REMOVED: from autocar_utils.utils import generate_target_course

import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector
import pyproj

class MapCanvas(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

    def wheelEvent(self, event):
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1.0 / zoom_factor, 1.0 / zoom_factor)
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.parent:
            self.parent.handle_mouse_click(event)
        super().mousePressEvent(event)


class LaneletClickPlanner(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Lanelet Click Planner')
        self.setGeometry(100, 100, 1200, 800)
        
        # ROS Node
        self.node = Node('lanelet_click_planner')

        # Parameters
        self.node.declare_parameter('map_frame', 'world')
        self.node.declare_parameter('map_origin.lat', 37.630117)
        self.node.declare_parameter('map_origin.lon', 127.081431)
        self.node.declare_parameter('lanelet2_map_path', '')
        # Allowed lanelet subtypes (if non-empty, selection will only accept these)
        self.node.declare_parameter('allowed_lanelet_subtypes', ['road'])
        # Maximum distance to snap selection to an allowed lanelet (meters)
        self.node.declare_parameter('max_select_distance', 3.0)
        # Smoothing params for /autocar/goals endpoint
        self.node.declare_parameter('smooth_goal_end', True)
        self.node.declare_parameter('smooth_goal_length_m', 1.0)
        self.node.declare_parameter('smooth_goal_pre_length_m', 0.5)
        self.node.declare_parameter('smooth_goal_samples', 0)  # 0 = keep original count
        self.map_frame = self.node.get_parameter('map_frame').get_parameter_value().string_value
        self.map_origin_lat = self.node.get_parameter('map_origin.lat').get_parameter_value().double_value
        self.map_origin_lon = self.node.get_parameter('map_origin.lon').get_parameter_value().double_value
        self.projector = UtmProjector(Origin(self.map_origin_lat, self.map_origin_lon))
        self.allowed_lanelet_subtypes = list(self.node.get_parameter('allowed_lanelet_subtypes').get_parameter_value().string_array_value)
        self.max_select_distance = self.node.get_parameter('max_select_distance').get_parameter_value().double_value
        self.smooth_goal_end = self.node.get_parameter('smooth_goal_end').get_parameter_value().bool_value
        self.smooth_goal_length_m = self.node.get_parameter('smooth_goal_length_m').get_parameter_value().double_value
        self.smooth_goal_pre_length_m = self.node.get_parameter('smooth_goal_pre_length_m').get_parameter_value().double_value
        self.smooth_goal_samples = self.node.get_parameter('smooth_goal_samples').get_parameter_value().integer_value

        # Remove map subscriber
        # self.map_sub = self.node.create_subscription(...)

        # Publishers
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.selected_lanelets_pub = self.node.create_publisher(MarkerArray, '/selected_lanelets_viz', qos_transient_local)
        self.way_pub = self.node.create_publisher(PoseArray, '/autocar/goals', 10)
        self.map_origin_pub = self.node.create_publisher(PointStamped, '/map/origin', qos_transient_local)

        # Node state
        self.lanelet_map = None
        self.selected_lanelet_ids = []

        # UI
        self.init_ui()

        # ROS spin timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(100) # 100ms
        
        # Ensure clean shutdown on SIGINT/SIGTERM
        self._setup_signal_handlers()
        
        # Load map directly
        self.load_map_from_file()
        
        # Graceful shutdown
        app = QApplication.instance()
        app.aboutToQuit.connect(self.on_quit)
        
        # Map loading flag
        self.map_loaded = False

    def _get_attr(self, attr_map, key: str) -> str:
        """Robustly fetch an attribute value from lanelet AttributeMap.
        Tries .get, dict access, and case-insensitive key scan. Returns empty string if missing."""
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
            # try case-insensitive scan
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

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Controls
        controls_layout = QHBoxLayout()
        self.publish_btn = QPushButton('Publish Waypoints')
        self.publish_btn.clicked.connect(self.publish_waypoints)
        self.clear_btn = QPushButton('Clear Selection')
        self.clear_btn.clicked.connect(self.clear_selection)
        controls_layout.addWidget(self.publish_btn)
        controls_layout.addWidget(self.clear_btn)
        
        # Info labels
        info_layout = QHBoxLayout()
        self.map_status_label = QLabel('Map: Not Loaded')
        self.selection_label = QLabel('Selected: 0 lanelets')
        info_layout.addWidget(self.map_status_label)
        info_layout.addWidget(self.selection_label)

        # Canvas
        self.map_canvas = MapCanvas(self)
        self.map_scene = QGraphicsScene()
        self.map_canvas.setScene(self.map_scene)

        layout.addLayout(controls_layout)
        layout.addLayout(info_layout)
        layout.addWidget(self.map_canvas)

    def spin_once(self):
        # If ROS context is already shutting down, stop timer and quit GUI to avoid RCLError
        if not rclpy.ok():
            try:
                self.node.get_logger().info('ROS context invalid. Stopping timer and quitting GUI...')
            except Exception:
                pass
            self._stop_and_quit()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        except Exception as e:
            try:
                self.node.get_logger().warn(f'spin_once exception: {e}. Quitting GUI...')
            except Exception:
                pass
            self._stop_and_quit()

    def on_quit(self):
        self.node.get_logger().info('Shutting down ROS node.')
        try:
            self.timer.stop()
        except Exception:
            pass
        self.node.destroy_node()
        # rclpy.shutdown() is handled in main

    # ───────────────── helpers ─────────────────
    def _setup_signal_handlers(self):
        def handler(sig, frame):
            try:
                self.node.get_logger().info(f'Received signal {sig}. Quitting...')
            except Exception:
                pass
            self._stop_and_quit()
        try:
            signal.signal(signal.SIGINT, handler)
            signal.signal(signal.SIGTERM, handler)
        except Exception:
            pass

    def _stop_and_quit(self):
        try:
            self.timer.stop()
        except Exception:
            pass
        app = QApplication.instance()
        if app is not None:
            app.quit()

    def load_map_from_file(self):
        map_path = self.node.get_parameter('lanelet2_map_path').get_parameter_value().string_value
        if not map_path:
            self.node.get_logger().error("'lanelet2_map_path' parameter is not set or empty.")
            self.map_status_label.setText('Map: Path not provided')
            return

        self.node.get_logger().info(f"Loading Lanelet2 map from file: {map_path}")
        self.map_status_label.setText('Map: Loading...')
        
        try:
            self.lanelet_map = lanelet2.io.load(map_path, self.projector)
            
            self.node.get_logger().info("Lanelet2 map loaded successfully.")
            self.map_status_label.setText(f'Map: Loaded ({len(self.lanelet_map.laneletLayer)} lanelets)')
            
            # Precompute selectable lanelet ids based on allowed subtypes (and exclude pedestrian/stop_line)
            allowed_subtypes_lower = set(s.lower() for s in (self.allowed_lanelet_subtypes or []))
            self.allowed_lanelet_ids = set()
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
            if not self.allowed_lanelet_ids:
                self.node.get_logger().warn('No lanelets match allowed subtypes. Click selection may be disabled. Check map tags (subtype=road).')
            
            # Publish map origin in UTM
            self.publish_map_origin()

            self.draw_map()
            self.map_loaded = True
        except Exception as e:
            self.node.get_logger().error(f"Failed to load lanelet map from file: {e}")
            self.map_status_label.setText('Map: Load Failed')


    def publish_map_origin(self):
        # This function converts the map's lat/lon origin to UTM coordinates and publishes it.
        # Using pyproj similar to the old planner for consistency. Assuming UTM zone 52.
        proj_utm = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        utm_x, utm_y = proj_utm(self.map_origin_lon, self.map_origin_lat)

        origin_msg = PointStamped()
        origin_msg.header.stamp = self.node.get_clock().now().to_msg()
        origin_msg.header.frame_id = 'utm' # A global, non-moving frame
        origin_msg.point.x = utm_x
        origin_msg.point.y = utm_y
        origin_msg.point.z = 0.0 # Assuming ground level

        self.map_origin_pub.publish(origin_msg)
        self.node.get_logger().info(f"Published map origin in UTM (x, y): ({utm_x}, {utm_y})")

    def draw_map(self):
        self.map_scene.clear()
        if not self.lanelet_map:
            return

        road_pen = QPen(QColor(100, 100, 100), 2)
        highlight_pen = QPen(QColor(0, 0, 255, 150), 3)

        # Draw only allowed lanelets (e.g., subtype == 'road')
        allowed_ids = getattr(self, 'allowed_lanelet_ids', set())
        for lanelet in self.lanelet_map.laneletLayer:
            if self.allowed_lanelet_subtypes and allowed_ids:
                if lanelet.id not in allowed_ids:
                    continue
            elif self.allowed_lanelet_subtypes:
                # Fallback: filter on-the-fly by attributes if allowed set not prepared
                subtype_val = self._get_attr(lanelet.attributes, 'subtype').lower()
                ped_val = self._get_attr(lanelet.attributes, 'participant:pedestrian').lower()
                ltype_val = self._get_attr(lanelet.attributes, 'type').lower()
                if subtype_val not in (s.lower() for s in self.allowed_lanelet_subtypes):
                    continue
                if ped_val == 'yes' or ltype_val == 'stop_line':
                    continue

            pen = highlight_pen if lanelet.id in self.selected_lanelet_ids else road_pen
            
            # Draw left bound
            left_bound_path = QPainterPath()
            for i, p in enumerate(lanelet.leftBound):
                if i == 0: left_bound_path.moveTo(p.x, -p.y)
                else: left_bound_path.lineTo(p.x, -p.y)
            self.map_scene.addPath(left_bound_path, pen)

            # Draw right bound
            right_bound_path = QPainterPath()
            for i, p in enumerate(lanelet.rightBound):
                if i == 0: right_bound_path.moveTo(p.x, -p.y)
                else: right_bound_path.lineTo(p.x, -p.y)
            self.map_scene.addPath(right_bound_path, pen)

        self.map_canvas.setSceneRect(self.map_scene.itemsBoundingRect())

    def handle_mouse_click(self, event):
        if not self.lanelet_map: return
        
        scene_pos = self.map_canvas.mapToScene(event.pos())
        point_ll = lanelet2.core.BasicPoint2d(scene_pos.x(), -scene_pos.y())

        # Find closest allowed lanelet
        k_neighbors = min(50, max(1, len(self.lanelet_map.laneletLayer)))
        nearest_list = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, point_ll, k_neighbors)
        if not nearest_list:
            return
        
        closest_lanelet = None
        closest_lanelet_dist = None
        for dist, cand_lanelet in nearest_list:
            # Always re-check attributes to be safe even if precomputed set is empty
            subtype = self._get_attr(cand_lanelet.attributes, 'subtype').lower()
            ped = self._get_attr(cand_lanelet.attributes, 'participant:pedestrian').lower()
            ltype = self._get_attr(cand_lanelet.attributes, 'type').lower()
            if self.allowed_lanelet_subtypes and subtype not in (s.lower() for s in self.allowed_lanelet_subtypes):
                continue
            if ped == 'yes':
                continue
            if ltype == 'stop_line':
                continue
            closest_lanelet = cand_lanelet
            closest_lanelet_dist = dist
            break

        if closest_lanelet is None:
            # No selectable lanelet near click
            self.node.get_logger().info('No selectable lanelet near click (filtered).')
            return
        
        # If the closest allowed lanelet is too far, ignore selection (prevents crosswalk clicks snapping to distant roads)
        if closest_lanelet_dist is not None and closest_lanelet_dist > self.max_select_distance:
            self.node.get_logger().info(f'Closest allowed lanelet is {closest_lanelet_dist:.2f} m away (> {self.max_select_distance:.2f} m). Ignoring click.')
            return
        
        # Toggle selection
        if closest_lanelet.id in self.selected_lanelet_ids:
            self.selected_lanelet_ids.remove(closest_lanelet.id)
        else:
            self.selected_lanelet_ids.append(closest_lanelet.id)
            
        self.node.get_logger().info(f"Selected IDs: {self.selected_lanelet_ids}")
        self.selection_label.setText(f'Selected: {len(self.selected_lanelet_ids)} lanelets')
        
        self.draw_map()
        self.publish_selected_lanelets_viz()

    def publish_selected_lanelets_viz(self):
        marker_array = MarkerArray()
        # Clear previous markers reliably
        delete_marker = Marker()
        delete_marker.header.stamp = self.node.get_clock().now().to_msg()
        delete_marker.ns = "selected_lanelets"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.selected_lanelets_pub.publish(marker_array)
        
        marker_array = MarkerArray()

        marker_id = 0
        for lanelet_id in self.selected_lanelet_ids:
            if self.lanelet_map.laneletLayer.exists(lanelet_id):
                lanelet = self.lanelet_map.laneletLayer[lanelet_id]
                
                base_marker = Marker()
                base_marker.header.frame_id = self.map_frame
                base_marker.header.stamp = self.node.get_clock().now().to_msg()
                base_marker.ns = "selected_lanelets"
                base_marker.type = Marker.LINE_STRIP
                base_marker.action = Marker.ADD
                base_marker.scale.x = 0.2
                base_marker.color.a = 0.8
                base_marker.color.r = 0.0
                base_marker.color.g = 0.5
                base_marker.color.b = 1.0 

                # Left Bound Marker
                left_marker = deepcopy(base_marker)
                left_marker.id = marker_id
                for p in lanelet.leftBound:
                    left_marker.points.append(Point(x=p.x, y=p.y, z=p.z))
                marker_array.markers.append(left_marker)
                marker_id += 1

                # Right Bound Marker
                right_marker = deepcopy(base_marker)
                right_marker.id = marker_id
                for p in lanelet.rightBound:
                    right_marker.points.append(Point(x=p.x, y=p.y, z=p.z))
                marker_array.markers.append(right_marker)
                marker_id += 1

        if marker_array.markers:
            self.selected_lanelets_pub.publish(marker_array)
    
    def _smooth_goal_tail(self, points):
        """
        Smooth the last `smooth_goal_length_m` of the assembled goal points using a local
        cubic spline fit. Optionally include `smooth_goal_pre_length_m` before the tail
        to ensure continuity. Keeps the number of points by default, unless
        `smooth_goal_samples` > 0 is provided.
        """
        try:
            if not points or len(points) < 4:
                return points

            # Build arrays and cumulative distances
            xs = [p.x for p in points]
            ys = [p.y for p in points]
            s_acc = [0.0]
            for i in range(1, len(points)):
                dx = xs[i] - xs[i-1]
                dy = ys[i] - ys[i-1]
                s_acc.append(s_acc[-1] + float(np.hypot(dx, dy)))

            total_len = s_acc[-1]
            if total_len <= max(0.1, float(self.smooth_goal_length_m)):
                return points

            tail_len = max(0.0, float(self.smooth_goal_length_m))
            pre_len = max(0.0, float(self.smooth_goal_pre_length_m))
            win_start_dist = max(0.0, total_len - tail_len - pre_len)

            # Find start index for smoothing window
            start_idx = 0
            for i, sv in enumerate(s_acc):
                if sv >= win_start_dist:
                    start_idx = i
                    break

            if len(points) - start_idx < 4:
                return points

            xs_w = xs[start_idx:]
            ys_w = ys[start_idx:]

            # Fit local spline and resample
            spline = CubicSpline2D(xs_w, ys_w)
            window_len = spline.s[-1]
            if window_len <= 1e-6:
                return points

            # Keep original number of points in the window unless overridden
            num_samples = len(xs_w) if int(self.smooth_goal_samples) <= 0 else int(self.smooth_goal_samples)
            ss = np.linspace(0.0, window_len, num_samples)

            # Prepare simple z interpolation over the window
            z_window = [points[i].z for i in range(start_idx, len(points))]
            z0 = z_window[0]
            z1 = z_window[-1]

            smoothed_tail = []
            for j, sj in enumerate(ss):
                sx, sy = spline.calc_position(sj)
                # Linear z across window (fallback if map has varying z)
                z = z0 + (z1 - z0) * (j / max(1, (len(ss) - 1)))
                smoothed_tail.append(Point(x=float(sx), y=float(sy), z=float(z)))

            # Reassemble points
            return points[:start_idx] + smoothed_tail
        except Exception as e:
            try:
                self.node.get_logger().warn(f"Tail smoothing failed: {e}. Publishing raw points.")
            except Exception:
                pass
            return points

    def publish_waypoints(self):
        self.node.get_logger().info("'Publish Waypoints' button clicked.")
        if not self.selected_lanelet_ids:
            self.node.get_logger().warn("No lanelets selected, cannot publish waypoints.")
            return
        
        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = self.node.get_clock().now().to_msg()
        
        all_points = []
        # This assumes the user clicks the lanelets in the correct driving order.
        for lanelet_id in self.selected_lanelet_ids:
            if self.lanelet_map.laneletLayer.exists(lanelet_id):
                lanelet = self.lanelet_map.laneletLayer[lanelet_id]
                centerline = lanelet.centerline
                for p in centerline:
                    # Avoid duplicates at lanelet connections
                    if not all_points or (all_points[-1].x != p.x or all_points[-1].y != p.y):
                        # For now, we only need x and y for path generation
                        all_points.append(Point(x=p.x, y=p.y, z=p.z))

        if len(all_points) < 2:
            self.node.get_logger().warn("Not enough unique points to form a path.")
            return

        # Optional: smooth the tail of the path to reduce spline jumps on route switches
        if self.smooth_goal_end:
            before_cnt = len(all_points)
            all_points = self._smooth_goal_tail(all_points)
            after_cnt = len(all_points)
            self.node.get_logger().info(f"Applied tail smoothing (points: {before_cnt} -> {after_cnt})")

        # Create a PoseArray from the collected (optionally smoothed) centerline points
        for i in range(len(all_points)):
            pose = Pose()
            pose.position = all_points[i]
            
            # Calculate orientation from the vector to the next point
            if i < len(all_points) - 1:
                dx = all_points[i+1].x - all_points[i].x
                dy = all_points[i+1].y - all_points[i].y
            else: # For the last point, use the previous vector
                dx = all_points[i].x - all_points[i-1].x
                dy = all_points[i].y - all_points[i-1].y
            
            yaw = np.arctan2(dy, dx)
            pose.orientation = yaw_to_quaternion(yaw)
            pose_array.poses.append(pose)
        
        if pose_array.poses:
            self.way_pub.publish(pose_array)
            self.node.get_logger().info(f"Successfully published {len(pose_array.poses)} raw waypoints to /autocar/goals topic.")
        else:
            self.node.get_logger().warn("No valid waypoints found to publish.")

    def clear_selection(self):
        self.selected_lanelet_ids = []
        self.node.get_logger().info("Selection cleared.")
        self.selection_label.setText('Selected: 0 lanelets')
        self.draw_map()
        self.publish_selected_lanelets_viz()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = LaneletClickPlanner()
    gui.show()
    
    app.exec_()
    
    # Cleanly shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()

