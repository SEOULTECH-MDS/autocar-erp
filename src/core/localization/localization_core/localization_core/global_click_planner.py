#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QGraphicsView, QGraphicsScene, QLabel, QHBoxLayout)
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QPainterPath, QPen, QColor, QPainter
from copy import deepcopy

from geometry_msgs.msg import PoseArray, Pose, Point, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from autoware_map_msgs.msg import LaneletMapBin

from autocar_utils.yaw_to_quaternion import yaw_to_quaternion
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
        self.map_frame = self.node.get_parameter('map_frame').get_parameter_value().string_value
        self.map_origin_lat = self.node.get_parameter('map_origin.lat').get_parameter_value().double_value
        self.map_origin_lon = self.node.get_parameter('map_origin.lon').get_parameter_value().double_value
        self.projector = UtmProjector(Origin(self.map_origin_lat, self.map_origin_lon))

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
        
        # Load map directly
        self.load_map_from_file()
        
        # Graceful shutdown
        app = QApplication.instance()
        app.aboutToQuit.connect(self.on_quit)
        
        # Map loading flag
        self.map_loaded = False

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
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def on_quit(self):
        self.node.get_logger().info('Shutting down ROS node.')
        self.node.destroy_node()
        # rclpy.shutdown() is handled in main

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

        for lanelet in self.lanelet_map.laneletLayer:
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

        # Find closest lanelet
        closest_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, point_ll, 1)
        if not closest_lanelets: return
        
        _, closest_lanelet = closest_lanelets[0]
        
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

        # Create a PoseArray from the collected, unique centerline points
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

