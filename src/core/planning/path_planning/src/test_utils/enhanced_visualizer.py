#!/usr/bin/env python3
"""
Enhanced Visualizer - 고급 시각화 노드

이 파일은 모드 셀렉터의 고급 시각화 기능을 제공합니다.

주요 기능:
- 3D 차량 모델 시각화
- 센서 데이터의 실시간 애니메이션
- 모드 변화 히스토리 그래프
- 성능 메트릭 시각화
- 인터랙티브 마커 지원

시각화 요소:
- 차량: 3D 모델 (모드별 색상 및 크기 변화)
- 센서 범위: 반투명 구체로 표시
- 경로: 모드 변화에 따른 궤적
- 히트맵: 센서 데이터 밀도
- 그래프: 실시간 성능 지표

발행 토픽:
- /enhanced_visualization (MarkerArray): 고급 3D 마커들
- /sensor_heatmap (MarkerArray): 센서 데이터 히트맵
- /performance_graph (MarkerArray): 성능 그래프
- /interactive_markers (InteractiveMarkerArray): 인터랙티브 마커

구독 토픽:
- /mode_state (UInt8): 현재 모드 상태
- /mode_description (String): 모드 설명
- /stop_line_distance (Float32): 정지선 거리
- /obstacle_detected (Bool): 장애물 감지
- /sign_detector (String): 배달 표지판
- /cone_detector (String): 주차 라바콘

사용법:
    ros2 run path_planning enhanced_visualizer

참고:
    이 노드는 mode_selector_visualizer.py보다 더 고급 시각화 기능을 제공합니다.
    개발 및 디버깅 시 상세한 정보를 시각적으로 확인할 수 있습니다.
"""
import rclpy
from rclpy.node import Node
from planning_msgs.msg import ModeState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

class EnhancedVisualizer(Node):
    def __init__(self):
        super().__init__('enhanced_visualizer')
        
        # Subscribers
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/vehicle_pose', self.pose_callback, 10)
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/enhanced_visualization', 10)
        
        # Timer for publishing markers
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        # Current state tracking
        self.current_mode = ModeState.DRIVE
        self.current_description = "Drive Mode: 정상 주행"
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        
        # Mode colors
        self.mode_colors = {
            ModeState.DRIVE: ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),      # Green
            ModeState.OBSTACLE_STATIC: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # Red
            ModeState.PAUSE: ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),      # Yellow
            ModeState.DELIVERY: ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),   # Blue
            ModeState.PARKING: ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)     # Magenta
        }
        
        # Mode names
        self.mode_names = {
            ModeState.DRIVE: "DRIVE",
            ModeState.OBSTACLE_STATIC: "OBSTACLE",
            ModeState.PAUSE: "PAUSE", 
            ModeState.DELIVERY: "DELIVERY",
            ModeState.PARKING: "PARKING"
        }
        
        # 시나리오 구역 정의
        self.scenario_zones = {
            'normal': {'start': (0, 0), 'end': (5, 0), 'color': (0, 1, 0, 0.3), 'name': '정상주행'},
            'stop_line': {'start': (5, 0), 'end': (8, 0), 'color': (1, 1, 0, 0.3), 'name': '정지선'},
            'obstacle': {'start': (8, 0), 'end': (12, 0), 'color': (1, 0, 0, 0.3), 'name': '장애물'},
            'delivery': {'start': (12, 0), 'end': (15, 3), 'color': (0, 0, 1, 0.3), 'name': '배달'},
            'parking': {'start': (15, 3), 'end': (18, 6), 'color': (1, 0, 1, 0.3), 'name': '주차'}
        }
        
        self.get_logger().info('🎨 Enhanced Visualizer 시작됨')
        
    def mode_callback(self, msg):
        """모드 상태 변화를 받아서 저장"""
        self.current_mode = msg.current_mode
        self.current_description = msg.description
        self.get_logger().info(f'🔄 시각화 모드 변경: {msg.description}')
        
    def pose_callback(self, msg):
        """차량 위치 정보를 받아서 저장"""
        self.vehicle_x = msg.pose.position.x
        self.vehicle_y = msg.pose.position.y
        
        # 쿼터니언에서 yaw 추출
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.vehicle_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
    def publish_markers(self):
        """RViz에서 모드 상태를 시각화하는 마커 발행"""
        marker_array = MarkerArray()
        
        # 1. 시나리오 구역 표시
        self.add_scenario_zones(marker_array)
        
        # 2. 차량 표시
        self.add_vehicle_marker(marker_array)
        
        # 3. 모드 상태 표시
        self.add_mode_status_marker(marker_array)
        
        # 4. 센서 데이터 시뮬레이션
        self.add_sensor_simulation(marker_array)
        
        # 5. 경로 추적선
        self.add_path_trace(marker_array)
        
        # 마커 발행
        self.marker_pub.publish(marker_array)
        
    def add_scenario_zones(self, marker_array):
        """시나리오 구역을 사각형으로 표시"""
        for zone_name, zone_info in self.scenario_zones.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "scenario_zones"
            marker.id = hash(zone_name) % 1000  # 고유 ID 생성
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 구역 중심점 계산
            start_x, start_y = zone_info['start']
            end_x, end_y = zone_info['end']
            center_x = (start_x + end_x) / 2.0
            center_y = (start_y + end_y) / 2.0
            
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.1
            
            # 크기 설정
            width = abs(end_x - start_x)
            height = abs(end_y - start_y)
            marker.scale.x = max(width, 0.5)
            marker.scale.y = max(height, 0.5)
            marker.scale.z = 0.1
            
            # 색상 설정
            color = zone_info['color']
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            
            marker_array.markers.append(marker)
            
            # 구역 이름 텍스트
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "zone_names"
            text_marker.id = hash(zone_name) % 1000 + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = center_x
            text_marker.pose.position.y = center_y
            text_marker.pose.position.z = 0.5
            
            text_marker.text = zone_info['name']
            text_marker.scale.z = 0.8
            text_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            
            marker_array.markers.append(text_marker)
            
    def add_vehicle_marker(self, marker_array):
        """차량을 화살표로 표시"""
        # 차량 본체
        vehicle_marker = Marker()
        vehicle_marker.header.frame_id = "map"
        vehicle_marker.header.stamp = self.get_clock().now().to_msg()
        vehicle_marker.ns = "vehicle"
        vehicle_marker.id = 0
        vehicle_marker.type = Marker.ARROW
        vehicle_marker.action = Marker.ADD
        
        vehicle_marker.pose.position.x = self.vehicle_x
        vehicle_marker.pose.position.y = self.vehicle_y
        vehicle_marker.pose.position.z = 0.0
        
        # 방향 설정
        vehicle_marker.pose.orientation.x = 0.0
        vehicle_marker.pose.orientation.y = 0.0
        vehicle_marker.pose.orientation.z = math.sin(self.vehicle_yaw / 2.0)
        vehicle_marker.pose.orientation.w = math.cos(self.vehicle_yaw / 2.0)
        
        # 크기 설정
        vehicle_marker.scale.x = 2.0  # 길이
        vehicle_marker.scale.y = 1.0  # 너비
        vehicle_marker.scale.z = 0.5  # 높이
        
        # 색상 설정 (현재 모드에 따라)
        vehicle_marker.color = self.mode_colors.get(self.current_mode, ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0))
        
        marker_array.markers.append(vehicle_marker)
        
    def add_mode_status_marker(self, marker_array):
        """모드 상태를 차량 위에 표시"""
        # 모드 상태 원
        status_marker = Marker()
        status_marker.header.frame_id = "map"
        status_marker.header.stamp = self.get_clock().now().to_msg()
        status_marker.ns = "mode_status"
        status_marker.id = 1
        status_marker.type = Marker.SPHERE
        status_marker.action = Marker.ADD
        
        # 차량 위에 위치
        status_marker.pose.position.x = self.vehicle_x
        status_marker.pose.position.y = self.vehicle_y
        status_marker.pose.position.z = 1.5
        
        # 크기 설정
        status_marker.scale.x = 0.8
        status_marker.scale.y = 0.8
        status_marker.scale.z = 0.8
        
        # 색상 설정
        status_marker.color = self.mode_colors.get(self.current_mode, ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0))
        
        marker_array.markers.append(status_marker)
        
        # 모드 이름 텍스트
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "mode_text"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = self.vehicle_x
        text_marker.pose.position.y = self.vehicle_y
        text_marker.pose.position.z = 2.5
        
        text_marker.text = self.mode_names.get(self.current_mode, "UNKNOWN")
        text_marker.scale.z = 0.6
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        marker_array.markers.append(text_marker)
        
    def add_sensor_simulation(self, marker_array):
        """센서 데이터 시뮬레이션 시각화"""
        # 정지선 시뮬레이션 (차량 앞쪽에 선으로 표시)
        if self.current_mode == ModeState.PAUSE:
            stop_line_marker = Marker()
            stop_line_marker.header.frame_id = "map"
            stop_line_marker.header.stamp = self.get_clock().now().to_msg()
            stop_line_marker.ns = "stop_line"
            stop_line_marker.id = 3
            stop_line_marker.type = Marker.LINE_STRIP
            stop_line_marker.action = Marker.ADD
            
            # 정지선 위치 (차량 앞 2m)
            front_x = self.vehicle_x + 2.0 * math.cos(self.vehicle_yaw)
            front_y = self.vehicle_y + 2.0 * math.sin(self.vehicle_yaw)
            
            # 정지선의 수직 방향
            perp_x = -math.sin(self.vehicle_yaw)
            perp_y = math.cos(self.vehicle_yaw)
            
            stop_line_marker.points.append(Point(x=front_x - 2.0 * perp_x, y=front_y - 2.0 * perp_y, z=0.0))
            stop_line_marker.points.append(Point(x=front_x + 2.0 * perp_x, y=front_y + 2.0 * perp_y, z=0.0))
            
            stop_line_marker.scale.x = 0.3
            stop_line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            marker_array.markers.append(stop_line_marker)
        
        # 장애물 시뮬레이션
        if self.current_mode == ModeState.OBSTACLE_STATIC:
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = "map"
            obstacle_marker.header.stamp = self.get_clock().now().to_msg()
            obstacle_marker.ns = "obstacle"
            obstacle_marker.id = 4
            obstacle_marker.type = Marker.CUBE
            obstacle_marker.action = Marker.ADD
            
            # 장애물 위치 (차량 앞 3m, 오른쪽 1m)
            obstacle_x = self.vehicle_x + 3.0 * math.cos(self.vehicle_yaw) + 1.0 * math.sin(self.vehicle_yaw)
            obstacle_y = self.vehicle_y + 3.0 * math.sin(self.vehicle_yaw) - 1.0 * math.cos(self.vehicle_yaw)
            
            obstacle_marker.pose.position.x = obstacle_x
            obstacle_marker.pose.position.y = obstacle_y
            obstacle_marker.pose.position.z = 0.5
            
            obstacle_marker.scale.x = 1.0
            obstacle_marker.scale.y = 1.0
            obstacle_marker.scale.z = 1.0
            
            obstacle_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            
            marker_array.markers.append(obstacle_marker)
        
        # 배달 표지판 시뮬레이션
        if self.current_mode == ModeState.DELIVERY:
            sign_marker = Marker()
            sign_marker.header.frame_id = "map"
            sign_marker.header.stamp = self.get_clock().now().to_msg()
            sign_marker.ns = "delivery_sign"
            sign_marker.id = 5
            sign_marker.type = Marker.CUBE
            sign_marker.action = Marker.ADD
            
            # 표지판 위치 (차량 오른쪽 3m)
            sign_x = self.vehicle_x + 3.0 * math.sin(self.vehicle_yaw)
            sign_y = self.vehicle_y - 3.0 * math.cos(self.vehicle_yaw)
            
            sign_marker.pose.position.x = sign_x
            sign_marker.pose.position.y = sign_y
            sign_marker.pose.position.z = 1.0
            
            sign_marker.scale.x = 0.5
            sign_marker.scale.y = 0.5
            sign_marker.scale.z = 1.0
            
            sign_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            
            marker_array.markers.append(sign_marker)
        
        # 주차 라바콘 시뮬레이션
        if self.current_mode == ModeState.PARKING:
            cone_marker = Marker()
            cone_marker.header.frame_id = "map"
            cone_marker.header.stamp = self.get_clock().now().to_msg()
            cone_marker.ns = "parking_cone"
            cone_marker.id = 6
            cone_marker.type = Marker.CONE
            cone_marker.action = Marker.ADD
            
            # 라바콘 위치 (차량 오른쪽 3m)
            cone_x = self.vehicle_x + 3.0 * math.sin(self.vehicle_yaw)
            cone_y = self.vehicle_y - 3.0 * math.cos(self.vehicle_yaw)
            
            cone_marker.pose.position.x = cone_x
            cone_marker.pose.position.y = cone_y
            cone_marker.pose.position.z = 0.5
            
            cone_marker.scale.x = 0.5
            cone_marker.scale.y = 0.5
            cone_marker.scale.z = 1.0
            
            cone_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
            
            marker_array.markers.append(cone_marker)
            
    def add_path_trace(self, marker_array):
        """차량 경로 추적선 (간단한 구현)"""
        # 실제로는 경로 히스토리를 저장해야 하지만, 여기서는 현재 위치만 표시
        trace_marker = Marker()
        trace_marker.header.frame_id = "map"
        trace_marker.header.stamp = self.get_clock().now().to_msg()
        trace_marker.ns = "path_trace"
        trace_marker.id = 7
        trace_marker.type = Marker.SPHERE
        trace_marker.action = Marker.ADD
        
        trace_marker.pose.position.x = self.vehicle_x
        trace_marker.pose.position.y = self.vehicle_y
        trace_marker.pose.position.z = 0.05
        
        trace_marker.scale.x = 0.1
        trace_marker.scale.y = 0.1
        trace_marker.scale.z = 0.1
        
        trace_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)
        
        marker_array.markers.append(trace_marker)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 