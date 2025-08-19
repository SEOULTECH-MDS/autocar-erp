#!/usr/bin/env python3
"""
Mission Map - 맵 기반 미션 시스템

이 파일은 맵에 미션을 배치하고 차량의 위치에 따라 미션을 인식하는 시스템입니다.

주요 기능:
- 맵에 미션 포인트 배치
- 차량 위치 기반 미션 인식
- 미션 완료 상태 관리
- 동적 센서 데이터 생성

미션 타입:
1. 정지선 (Stop Line)
2. 장애물 (Obstacle)
3. 배달 표지판 (Delivery Sign)
4. 주차 라바콘 (Parking Cone)
5. 일반 주행 구간 (Normal Driving)

작성자: BAE Team
버전: 1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, UInt8
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import math
import time

class MissionPoint:
    def __init__(self, x, y, mission_type, params=None):
        self.x = x
        self.y = y
        self.mission_type = mission_type
        self.params = params or {}
        self.completed = False
        self.activation_distance = 3.0  # 미션 인식 거리
        
    def is_near(self, vehicle_x, vehicle_y):
        """차량이 미션 포인트 근처에 있는지 확인"""
        distance = math.sqrt((vehicle_x - self.x)**2 + (vehicle_y - self.y)**2)
        return distance <= self.activation_distance
        
    def get_sensor_data(self, vehicle_x, vehicle_y):
        """차량 위치에 따른 센서 데이터 반환"""
        if not self.is_near(vehicle_x, vehicle_y):
            return None
            
        distance = math.sqrt((vehicle_x - self.x)**2 + (vehicle_y - self.y)**2)
        
        if self.mission_type == "stop_line":
            return {
                'stop_line_distance': distance,
                'obstacle_detected': False,
                'sign_detector': '',
                'cone_detector': ''
            }
        elif self.mission_type == "obstacle":
            return {
                'stop_line_distance': 50.0,
                'obstacle_detected': True,
                'sign_detector': '',
                'cone_detector': ''
            }
        elif self.mission_type == "delivery_sign":
            return {
                'stop_line_distance': 50.0,
                'obstacle_detected': False,
                'sign_detector': 'DELIVERY',
                'cone_detector': ''
            }
        elif self.mission_type == "parking_cone":
            return {
                'stop_line_distance': 50.0,
                'obstacle_detected': False,
                'sign_detector': '',
                'cone_detector': 'PARKING'
            }
        else:  # normal_driving
            return {
                'stop_line_distance': 50.0,
                'obstacle_detected': False,
                'sign_detector': '',
                'cone_detector': ''
            }

class MissionMap(Node):
    def __init__(self):
        super().__init__('mission_map')
        
        # Publishers for sensor data
        self.stop_line_pub = self.create_publisher(Float32, '/stop_line_distance', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.sign_pub = self.create_publisher(String, '/sign_detector', 10)
        self.cone_pub = self.create_publisher(String, '/cone_detector', 10)
        
        # Publishers for visualization
        self.mission_markers_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)
        self.vehicle_path_pub = self.create_publisher(Path, '/vehicle_path', 10)
        
        # Subscriber for mode state
        self.mode_sub = self.create_subscription(UInt8, '/mode_state', self.mode_callback, 10)
        
        # Timer for updating
        self.timer = self.create_timer(0.1, self.update_mission)  # 10Hz
        
        # Mission map setup
        self.setup_mission_map()
        
        # Vehicle state
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_speed = 2.0  # m/s
        self.current_mode = 0
        
        # Path tracking
        self.vehicle_path = Path()
        self.vehicle_path.header.frame_id = "map"
        
        # Mission completion tracking
        self.completed_missions = []
        
        self.get_logger().info('🗺️ Mission Map 시작됨')
        
    def setup_mission_map(self):
        """미션 맵 설정"""
        self.missions = [
            # 시작점에서 정상 주행
            MissionPoint(0, 0, "normal_driving"),
            
            # 10m 지점에 정지선
            MissionPoint(10, 0, "stop_line"),
            
            # 20m 지점에 장애물
            MissionPoint(20, 2, "obstacle"),
            
            # 30m 지점에 배달 표지판
            MissionPoint(30, 0, "delivery_sign"),
            
            # 40m 지점에 주차 라바콘
            MissionPoint(40, 0, "parking_cone"),
            
            # 50m 지점에 정상 주행
            MissionPoint(50, 0, "normal_driving"),
        ]
        
        self.get_logger().info(f'📋 {len(self.missions)}개의 미션 포인트 설정됨')
        
    def mode_callback(self, msg):
        """모드 상태 수신"""
        self.current_mode = msg.data
        
    def update_mission(self):
        """미션 업데이트 및 센서 데이터 발행"""
        # 차량 위치 업데이트 (간단한 직선 주행)
        self.vehicle_x += self.vehicle_speed * 0.1  # 0.1초 간격
        
        # 경로 기록
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.vehicle_x
        pose.pose.position.y = self.vehicle_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.vehicle_path.poses.append(pose)
        
        # 경로 발행
        self.vehicle_path.header.stamp = self.get_clock().now().to_msg()
        self.vehicle_path_pub.publish(self.vehicle_path)
        
        # 현재 활성화된 미션 찾기
        active_mission = None
        for mission in self.missions:
            if mission.is_near(self.vehicle_x, self.vehicle_y) and not mission.completed:
                active_mission = mission
                break
        
        # 센서 데이터 발행
        if active_mission:
            sensor_data = active_mission.get_sensor_data(self.vehicle_x, self.vehicle_y)
            if sensor_data:
                # 정지선 거리
                stop_line_msg = Float32()
                stop_line_msg.data = sensor_data['stop_line_distance']
                self.stop_line_pub.publish(stop_line_msg)
                
                # 장애물 감지
                obstacle_msg = Bool()
                obstacle_msg.data = sensor_data['obstacle_detected']
                self.obstacle_pub.publish(obstacle_msg)
                
                # 배달 표지판
                sign_msg = String()
                sign_msg.data = sensor_data['sign_detector']
                self.sign_pub.publish(sign_msg)
                
                # 주차 라바콘
                cone_msg = String()
                cone_msg.data = sensor_data['cone_detector']
                self.cone_pub.publish(cone_msg)
                
                # 미션 완료 체크 (모드가 변경되면)
                if self.current_mode in [2, 3, 4]:  # Pause, Delivery, Parking 모드
                    if not active_mission.completed:
                        active_mission.completed = True
                        self.completed_missions.append(active_mission)
                        self.get_logger().info(f'✅ 미션 완료: {active_mission.mission_type} at ({active_mission.x:.1f}, {active_mission.y:.1f})')
        else:
            # 기본 센서 데이터 (정상 주행)
            stop_line_msg = Float32()
            stop_line_msg.data = 50.0
            self.stop_line_pub.publish(stop_line_msg)
            
            obstacle_msg = Bool()
            obstacle_msg.data = False
            self.obstacle_pub.publish(obstacle_msg)
            
            sign_msg = String()
            sign_msg.data = ''
            self.sign_pub.publish(sign_msg)
            
            cone_msg = String()
            cone_msg.data = ''
            self.cone_pub.publish(cone_msg)
        
        # 미션 마커 시각화
        self.publish_mission_markers()
        
        # 5초마다 상태 출력
        if int(time.time() * 10) % 50 == 0:
            self.print_status()
            
    def publish_mission_markers(self):
        """미션 포인트들을 3D 마커로 시각화"""
        marker_array = MarkerArray()
        
        for i, mission in enumerate(self.missions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mission_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(mission.x)
            marker.pose.position.y = float(mission.y)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # 미션 타입에 따른 색상
            if mission.completed:
                # 완료된 미션: 회색
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 0.3
            elif mission.is_near(self.vehicle_x, self.vehicle_y):
                # 활성화된 미션: 노란색
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                # 대기 중인 미션: 미션 타입별 색상
                if mission.mission_type == "stop_line":
                    marker.color.r = 1.0  # 빨간색
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif mission.mission_type == "obstacle":
                    marker.color.r = 0.0  # 파란색
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif mission.mission_type == "delivery_sign":
                    marker.color.r = 1.0  # 보라색
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif mission.mission_type == "parking_cone":
                    marker.color.r = 0.0  # 주황색
                    marker.color.g = 0.5
                    marker.color.b = 1.0
                else:  # normal_driving
                    marker.color.r = 0.0  # 초록색
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                marker.color.a = 0.6
                
            marker_array.markers.append(marker)
            
            # 미션 타입 텍스트
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "mission_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(mission.x)
            text_marker.pose.position.y = float(mission.y)
            text_marker.pose.position.z = 1.5
            text_marker.pose.orientation.w = 1.0
            
            text_marker.text = mission.mission_type.replace('_', ' ').title()
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.mission_markers_pub.publish(marker_array)
        
    def print_status(self):
        """현재 상태 출력"""
        completed_count = len(self.completed_missions)
        total_count = len(self.missions)
        self.get_logger().info(f'📍 차량 위치: ({self.vehicle_x:.1f}, {self.vehicle_y:.1f}) | 완료된 미션: {completed_count}/{total_count}')

def main(args=None):
    rclpy.init(args=args)
    mission_map = MissionMap()
    
    try:
        rclpy.spin(mission_map)
    except KeyboardInterrupt:
        mission_map.get_logger().info('🗺️ Mission Map 종료됨')
    finally:
        mission_map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 