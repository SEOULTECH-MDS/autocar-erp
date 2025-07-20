#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from planning_msgs.msg import ModeState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

class ModeSelectorVisualizer(Node):
    def __init__(self):
        super().__init__('mode_selector_visualizer')
        
        # Subscriber for mode state
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_callback, 10)
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/mode_visualization', 10)
        
        # Timer for publishing markers
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        # Current mode tracking
        self.current_mode = ModeState.DRIVE
        self.current_description = "Drive Mode: 정상 주행"
        
        # Mode colors
        self.mode_colors = {
            ModeState.DRIVE: ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),      # Green
            ModeState.OBSTACLE_STATIC: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red
            ModeState.PAUSE: ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),      # Yellow
            ModeState.DELIVERY: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),   # Blue
            ModeState.PARKING: ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)     # Magenta
        }
        
        # Mode names
        self.mode_names = {
            ModeState.DRIVE: "DRIVE",
            ModeState.OBSTACLE_STATIC: "OBSTACLE",
            ModeState.PAUSE: "PAUSE", 
            ModeState.DELIVERY: "DELIVERY",
            ModeState.PARKING: "PARKING"
        }
        
        self.get_logger().info('Mode Selector Visualizer 시작됨')
        
    def mode_callback(self, msg):
        """모드 상태 변화를 받아서 저장"""
        self.current_mode = msg.current_mode
        self.current_description = msg.description
        self.get_logger().info(f'🔄 시각화 모드 변경: {msg.description}')
        
    def publish_markers(self):
        """RViz에서 모드 상태를 시각화하는 마커 발행"""
        marker_array = MarkerArray()
        
        # 1. 모드 상태 표시 원형 마커
        mode_marker = Marker()
        mode_marker.header.frame_id = "map"
        mode_marker.header.stamp = self.get_clock().now().to_msg()
        mode_marker.ns = "mode_status"
        mode_marker.id = 0
        mode_marker.type = Marker.CYLINDER
        mode_marker.action = Marker.ADD
        
        # 위치 설정 (차량 앞쪽)
        mode_marker.pose.position.x = 5.0
        mode_marker.pose.position.y = 0.0
        mode_marker.pose.position.z = 1.0
        
        # 크기 설정
        mode_marker.scale.x = 2.0
        mode_marker.scale.y = 2.0
        mode_marker.scale.z = 0.5
        
        # 색상 설정
        mode_marker.color = self.mode_colors.get(self.current_mode, ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0))
        
        marker_array.markers.append(mode_marker)
        
        # 2. 모드 이름 텍스트 마커
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "mode_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # 위치 설정
        text_marker.pose.position.x = 5.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 2.0
        
        # 텍스트 설정
        text_marker.text = self.mode_names.get(self.current_mode, "UNKNOWN")
        
        # 크기 설정
        text_marker.scale.z = 1.0
        
        # 색상 설정 (흰색)
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        marker_array.markers.append(text_marker)
        
        # 3. 모드 설명 텍스트 마커
        desc_marker = Marker()
        desc_marker.header.frame_id = "map"
        desc_marker.header.stamp = self.get_clock().now().to_msg()
        desc_marker.ns = "mode_description"
        desc_marker.id = 2
        desc_marker.type = Marker.TEXT_VIEW_FACING
        desc_marker.action = Marker.ADD
        
        # 위치 설정
        desc_marker.pose.position.x = 5.0
        desc_marker.pose.position.y = 0.0
        desc_marker.pose.position.z = 3.0
        
        # 텍스트 설정 (한글 설명)
        desc_marker.text = self.current_description
        
        # 크기 설정 (더 작게)
        desc_marker.scale.z = 0.5
        
        # 색상 설정 (노란색)
        desc_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        
        marker_array.markers.append(desc_marker)
        
        # 4. 센서 데이터 시뮬레이션 시각화
        self.add_sensor_visualization(marker_array)
        
        # 마커 발행
        self.marker_pub.publish(marker_array)
        
    def add_sensor_visualization(self, marker_array):
        """센서 데이터 시뮬레이션을 시각화"""
        # 정지선 거리 시뮬레이션 (차량 앞쪽에 선으로 표시)
        stop_line_marker = Marker()
        stop_line_marker.header.frame_id = "map"
        stop_line_marker.header.stamp = self.get_clock().now().to_msg()
        stop_line_marker.ns = "stop_line_simulation"
        stop_line_marker.id = 3
        stop_line_marker.type = Marker.LINE_STRIP
        stop_line_marker.action = Marker.ADD
        
        # 정지선 위치 (차량 앞 3m)
        stop_line_marker.points.append(Point(x=3.0, y=-2.0, z=0.0))
        stop_line_marker.points.append(Point(x=3.0, y=2.0, z=0.0))
        
        # 선 스타일 설정
        stop_line_marker.scale.x = 0.2
        stop_line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 빨간색
        
        marker_array.markers.append(stop_line_marker)
        
        # 장애물 시뮬레이션 (점으로 표시)
        if self.current_mode == ModeState.OBSTACLE_STATIC:
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = "map"
            obstacle_marker.header.stamp = self.get_clock().now().to_msg()
            obstacle_marker.ns = "obstacle_simulation"
            obstacle_marker.id = 4
            obstacle_marker.type = Marker.SPHERE
            obstacle_marker.action = Marker.ADD
            
            # 장애물 위치
            obstacle_marker.pose.position.x = 2.0
            obstacle_marker.pose.position.y = 1.0
            obstacle_marker.pose.position.z = 0.5
            
            # 크기 설정
            obstacle_marker.scale.x = 1.0
            obstacle_marker.scale.y = 1.0
            obstacle_marker.scale.z = 1.0
            
            # 색상 설정
            obstacle_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            
            marker_array.markers.append(obstacle_marker)
        
        # 배달 표지판 시뮬레이션
        if self.current_mode == ModeState.DELIVERY:
            sign_marker = Marker()
            sign_marker.header.frame_id = "map"
            sign_marker.header.stamp = self.get_clock().now().to_msg()
            sign_marker.ns = "delivery_sign_simulation"
            sign_marker.id = 5
            sign_marker.type = Marker.CUBE
            sign_marker.action = Marker.ADD
            
            # 표지판 위치
            sign_marker.pose.position.x = 4.0
            sign_marker.pose.position.y = 3.0
            sign_marker.pose.position.z = 1.0
            
            # 크기 설정
            sign_marker.scale.x = 0.5
            sign_marker.scale.y = 0.5
            sign_marker.scale.z = 1.0
            
            # 색상 설정
            sign_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            
            marker_array.markers.append(sign_marker)
        
        # 주차 라바콘 시뮬레이션
        if self.current_mode == ModeState.PARKING:
            cone_marker = Marker()
            cone_marker.header.frame_id = "map"
            cone_marker.header.stamp = self.get_clock().now().to_msg()
            cone_marker.ns = "parking_cone_simulation"
            cone_marker.id = 6
            cone_marker.type = Marker.CONE
            cone_marker.action = Marker.ADD
            
            # 라바콘 위치
            cone_marker.pose.position.x = 4.0
            cone_marker.pose.position.y = -3.0
            cone_marker.pose.position.z = 0.5
            
            # 크기 설정
            cone_marker.scale.x = 0.5
            cone_marker.scale.y = 0.5
            cone_marker.scale.z = 1.0
            
            # 색상 설정
            cone_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
            
            marker_array.markers.append(cone_marker)

def main(args=None):
    rclpy.init(args=args)
    visualizer = ModeSelectorVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        visualizer.get_logger().info('시각화 노드 종료')
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 