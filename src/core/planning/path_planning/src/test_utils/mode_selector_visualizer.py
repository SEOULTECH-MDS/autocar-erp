#!/usr/bin/env python3
"""
Mode Selector Visualizer - RViz 시각화 노드

이 파일은 모드 셀렉터의 상태와 센서 데이터를 RViz에서 3D로 시각화합니다.

주요 기능:
- 차량, 정지선, 장애물, 표지판, 라바콘을 3D 마커로 표시
- 현재 모드 상태를 텍스트 오버레이로 표시
- 센서 데이터를 실시간으로 시각화
- 모드 변화 히스토리를 경로로 표시

구독 토픽:
- /mode_state (UInt8): 현재 모드 상태
- /mode_description (String): 모드 설명
- /stop_line_distance (Float32): 정지선 거리
- /obstacle_detected (Bool): 장애물 감지
- /sign_detector (String): 배달 표지판
- /cone_detector (String): 주차 라바콘

발행 토픽:
- /mode_visualization (MarkerArray): 3D 마커 배열
- /mode_text (Marker): 텍스트 오버레이
- /mode_text_array (MarkerArray): 텍스트 마커 배열
- /mode_history_path (Path): 모드 변화 경로

사용법:
    ros2 run path_planning mode_selector_visualizer
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, String, Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import time

class ModeSelectorVisualizer(Node):
    def __init__(self):
        super().__init__('mode_selector_visualizer')
        
        # Publishers for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/mode_visualization', 10)
        self.text_marker_pub = self.create_publisher(Marker, '/mode_text', 10)
        self.path_pub = self.create_publisher(Path, '/mode_history_path', 10)
        
        # Subscribers for mode and sensor data
        self.mode_sub = self.create_subscription(UInt8, '/mode_state', self.mode_callback, 10)
        self.desc_sub = self.create_subscription(String, '/mode_description', self.desc_callback, 10)
        self.stop_line_sub = self.create_subscription(Float32, '/stop_line_distance', self.stop_line_callback, 10)
        self.obstacle_sub = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.sign_sub = self.create_subscription(String, '/sign_detector', self.sign_callback, 10)
        self.cone_sub = self.create_subscription(String, '/cone_detector', self.cone_callback, 10)
        
        # Timer for visualization updates
        self.timer = self.create_timer(0.1, self.update_visualization)  # 10Hz
        
        # Visualization state
        self.current_mode = 0
        self.current_description = "UNKNOWN"
        self.stop_line_distance = 50.0
        self.obstacle_detected = False
        self.sign_detected = ""
        self.cone_detected = ""
        
        # Mode history for path visualization
        self.mode_history = []
        self.start_time = time.time()
        
        # Color definitions for different modes
        self.mode_colors = {
            0: (0.0, 1.0, 0.0, 0.8),    # DRIVE: Green
            1: (1.0, 0.0, 0.0, 0.8),    # OBSTACLE: Red
            2: (1.0, 1.0, 0.0, 0.8),    # PAUSE: Yellow
            3: (0.0, 0.0, 1.0, 0.8),    # DELIVERY: Blue
            4: (1.0, 0.0, 1.0, 0.8),    # PARKING: Magenta
        }
        
        self.get_logger().info('🎨 Mode Selector Visualizer 시작됨')
        
    def mode_callback(self, msg):
        """모드 상태 수신"""
        if msg.data != self.current_mode:
            self.current_mode = msg.data
            current_time = time.time() - self.start_time
            self.mode_history.append({
                'time': current_time,
                'mode': msg.data,
                'description': self.current_description
            })
            self.get_logger().info(f'🔄 모드 변경: {self.current_description} (모드: {msg.data})')
            
    def desc_callback(self, msg):
        """모드 설명 수신"""
        self.current_description = msg.data
        
    def stop_line_callback(self, msg):
        """정지선 거리 수신"""
        self.stop_line_distance = msg.data
        
    def obstacle_callback(self, msg):
        """장애물 감지 수신"""
        self.obstacle_detected = msg.data
        
    def sign_callback(self, msg):
        """배달 표지판 수신"""
        self.sign_detected = msg.data
        
    def cone_callback(self, msg):
        """주차 라바콘 수신"""
        self.cone_detected = msg.data
        
    def update_visualization(self):
        """RViz 시각화 업데이트"""
        self.publish_mode_text()
        self.publish_sensor_markers()
        self.publish_mode_history_path()
        
    def publish_mode_text(self):
        """모드 상태를 텍스트로 표시"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mode_text"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (차량 앞쪽에 표시)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0
        
        # Text content
        marker.text = f"Mode: {self.current_description}\nStop Line: {self.stop_line_distance:.1f}m\nObstacle: {self.obstacle_detected}\nSign: {self.sign_detected}\nCone: {self.cone_detected}"
        
        # Text properties
        marker.scale.z = 0.5  # Text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        self.text_marker_pub.publish(marker)
        
    def publish_sensor_markers(self):
        """센서 데이터를 마커로 표시"""
        marker_array = MarkerArray()
        
        # 1. 차량 마커 (현재 모드에 따른 색상)
        vehicle_marker = Marker()
        vehicle_marker.header.frame_id = "map"
        vehicle_marker.header.stamp = self.get_clock().now().to_msg()
        vehicle_marker.ns = "vehicle"
        vehicle_marker.id = 0
        vehicle_marker.type = Marker.CUBE
        vehicle_marker.action = Marker.ADD
        
        vehicle_marker.pose.position.x = 0.0
        vehicle_marker.pose.position.y = 0.0
        vehicle_marker.pose.position.z = 0.5
        vehicle_marker.pose.orientation.w = 1.0
        
        vehicle_marker.scale.x = 2.0  # 차량 길이
        vehicle_marker.scale.y = 1.0  # 차량 폭
        vehicle_marker.scale.z = 1.0  # 차량 높이
        
        # 모드에 따른 색상 설정
        if self.current_mode in self.mode_colors:
            r, g, b, a = self.mode_colors[self.current_mode]
            vehicle_marker.color.r = r
            vehicle_marker.color.g = g
            vehicle_marker.color.b = b
            vehicle_marker.color.a = a
        else:
            vehicle_marker.color.r = 0.5
            vehicle_marker.color.g = 0.5
            vehicle_marker.color.b = 0.5
            vehicle_marker.color.a = 0.8
            
        marker_array.markers.append(vehicle_marker)
        
        # 2. 정지선 마커
        if self.stop_line_distance < 10.0:
            stop_line_marker = Marker()
            stop_line_marker.header.frame_id = "map"
            stop_line_marker.header.stamp = self.get_clock().now().to_msg()
            stop_line_marker.ns = "stop_line"
            stop_line_marker.id = 1
            stop_line_marker.type = Marker.LINE_STRIP
            stop_line_marker.action = Marker.ADD
            
            # 정지선 위치 (차량 앞쪽)
            stop_line_marker.pose.orientation.w = 1.0
            
            # 정지선 점들
            for i in range(-5, 6):
                point = Point()
                point.x = self.stop_line_distance
                point.y = i * 0.5
                point.z = 0.0
                stop_line_marker.points.append(point)
                
            stop_line_marker.scale.x = 0.2  # 선 두께
            stop_line_marker.color.r = 1.0
            stop_line_marker.color.g = 0.0
            stop_line_marker.color.b = 0.0
            stop_line_marker.color.a = 0.8
            
            marker_array.markers.append(stop_line_marker)
            
        # 3. 장애물 마커
        if self.obstacle_detected:
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = "map"
            obstacle_marker.header.stamp = self.get_clock().now().to_msg()
            obstacle_marker.ns = "obstacle"
            obstacle_marker.id = 2
            obstacle_marker.type = Marker.SPHERE
            obstacle_marker.action = Marker.ADD
            
            obstacle_marker.pose.position.x = 5.0  # 차량 앞 5m
            obstacle_marker.pose.position.y = 0.0
            obstacle_marker.pose.position.z = 0.5
            obstacle_marker.pose.orientation.w = 1.0
            
            obstacle_marker.scale.x = 1.0
            obstacle_marker.scale.y = 1.0
            obstacle_marker.scale.z = 1.0
            
            obstacle_marker.color.r = 1.0
            obstacle_marker.color.g = 0.0
            obstacle_marker.color.b = 0.0
            obstacle_marker.color.a = 0.8
            
            marker_array.markers.append(obstacle_marker)
            
        # 4. 배달 표지판 마커
        if self.sign_detected:
            sign_marker = Marker()
            sign_marker.header.frame_id = "map"
            sign_marker.header.stamp = self.get_clock().now().to_msg()
            sign_marker.ns = "delivery_sign"
            sign_marker.id = 3
            sign_marker.type = Marker.CYLINDER
            sign_marker.action = Marker.ADD
            
            sign_marker.pose.position.x = 3.0
            sign_marker.pose.position.y = 2.0
            sign_marker.pose.position.z = 1.0
            sign_marker.pose.orientation.w = 1.0
            
            sign_marker.scale.x = 0.5
            sign_marker.scale.y = 0.5
            sign_marker.scale.z = 2.0
            
            sign_marker.color.r = 0.0
            sign_marker.color.g = 0.0
            sign_marker.color.b = 1.0
            sign_marker.color.a = 0.8
            
            marker_array.markers.append(sign_marker)
            
        # 5. 주차 라바콘 마커
        if self.cone_detected:
            cone_marker = Marker()
            cone_marker.header.frame_id = "map"
            cone_marker.header.stamp = self.get_clock().now().to_msg()
            cone_marker.ns = "parking_cone"
            cone_marker.id = 4
            cone_marker.type = Marker.CYLINDER
            cone_marker.action = Marker.ADD
            
            cone_marker.pose.position.x = 3.0
            cone_marker.pose.position.y = -2.0
            cone_marker.pose.position.z = 0.5
            cone_marker.pose.orientation.w = 1.0
            
            cone_marker.scale.x = 0.5
            cone_marker.scale.y = 0.5
            cone_marker.scale.z = 1.0
            
            cone_marker.color.r = 1.0
            cone_marker.color.g = 0.0
            cone_marker.color.b = 1.0
            cone_marker.color.a = 0.8
            
            marker_array.markers.append(cone_marker)
            
        self.marker_pub.publish(marker_array)
        
    def publish_mode_history_path(self):
        """모드 변경 히스토리를 경로로 표시"""
        if len(self.mode_history) < 2:
            return
            
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i, record in enumerate(self.mode_history):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # 시간에 따른 X 위치, 모드에 따른 Y 위치
            pose.pose.position.x = record['time'] * 0.5  # 시간을 거리로 변환
            pose.pose.position.y = record['mode'] * 2.0  # 모드를 높이로 변환
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
            
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    visualizer = ModeSelectorVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        visualizer.get_logger().info('🎨 Visualizer 종료됨')
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 