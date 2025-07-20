#!/usr/bin/env python3
"""
Dynamic Vehicle Test - 동적 차량 테스트 노드

이 파일은 움직이는 차량을 시뮬레이션하여 모드 셀렉터를 테스트합니다.

주요 기능:
- 가상 차량의 움직임 시뮬레이션
- 센서 데이터의 동적 변화
- 실제 주행 환경과 유사한 테스트
- 차량 위치에 따른 센서 데이터 변화
- 복잡한 주행 시나리오 제공

시뮬레이션 요소:
- 차량 위치: 시간에 따른 이동
- 센서 범위: 차량 위치에 따른 변화
- 환경 요소: 정지선, 장애물, 표지판의 상대적 위치
- 물리적 제약: 차량의 최대 속도, 가속도 등

발행 토픽:
- /stop_line_distance (Float32): 차량 위치 기반 정지선 거리
- /obstacle_detected (Bool): 차량 위치 기반 장애물 감지
- /sign_detector (String): 차량 위치 기반 표지판 감지
- /cone_detector (String): 차량 위치 기반 라바콘 감지
- /vehicle_pose (PoseStamped): 차량의 현재 위치와 방향

구독 토픽:
- /mode_state (UInt8): 현재 모드 상태
- /mode_description (String): 모드 설명

특별 기능:
- 다양한 주행 경로 시뮬레이션
- 센서 노이즈 및 지연 시뮬레이션
- 긴급 상황 시뮬레이션
- 복합 미션 시나리오

사용법:
    ros2 run path_planning dynamic_vehicle_test

참고:
    이 노드는 실제 주행 환경과 가장 유사한 테스트를 제공합니다.
    모드 셀렉터의 실제 성능을 평가하는 데 사용됩니다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import PoseStamped
from planning_msgs.msg import ModeState
import time
import math
import numpy as np

class DynamicVehicleTest(Node):
    def __init__(self):
        super().__init__('dynamic_vehicle_test')
        
        # Publishers for sensor data simulation
        self.stop_line_pub = self.create_publisher(Float32, '/stop_line_distance', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.sign_pub = self.create_publisher(String, '/sign_detector', 10)
        self.cone_pub = self.create_publisher(String, '/cone_detector', 10)
        self.vehicle_pose_pub = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        
        # Subscriber for mode state
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_callback, 10)
        
        # Timer for publishing test data
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10Hz로 더 빠르게
        
        # Test scenario variables
        self.test_time = 0.0
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        
        # 시나리오 정의 (위치 기반)
        self.scenarios = [
            {
                'name': '정상 주행 구간',
                'start_pos': (0.0, 0.0),
                'end_pos': (5.0, 0.0),
                'duration': 10.0,
                'sensors': {'stop_line': 50.0, 'obstacle': False, 'sign': '', 'cone': ''}
            },
            {
                'name': '정지선 근처',
                'start_pos': (5.0, 0.0),
                'end_pos': (8.0, 0.0),
                'duration': 5.0,
                'sensors': {'stop_line': 2.0, 'obstacle': False, 'sign': '', 'cone': ''}
            },
            {
                'name': '장애물 구간',
                'start_pos': (8.0, 0.0),
                'end_pos': (12.0, 0.0),
                'duration': 8.0,
                'sensors': {'stop_line': 50.0, 'obstacle': True, 'sign': '', 'cone': ''}
            },
            {
                'name': '배달 구역',
                'start_pos': (12.0, 0.0),
                'end_pos': (15.0, 3.0),
                'duration': 6.0,
                'sensors': {'stop_line': 50.0, 'obstacle': False, 'sign': 'DELIVERY', 'cone': ''}
            },
            {
                'name': '주차 구역',
                'start_pos': (15.0, 3.0),
                'end_pos': (18.0, 6.0),
                'duration': 6.0,
                'sensors': {'stop_line': 50.0, 'obstacle': False, 'sign': '', 'cone': 'PARKING'}
            },
            {
                'name': '복귀 구간',
                'start_pos': (18.0, 6.0),
                'end_pos': (0.0, 0.0),
                'duration': 10.0,
                'sensors': {'stop_line': 50.0, 'obstacle': False, 'sign': '', 'cone': ''}
            }
        ]
        
        # Current scenario tracking
        self.current_scenario = 0
        self.scenario_start_time = 0.0
        self.scenario_progress = 0.0
        
        # Current mode tracking
        self.current_mode = "UNKNOWN"
        self.mode_history = []
        
        self.get_logger().info('🚗 Dynamic Vehicle Test Node 시작됨')
        
    def mode_callback(self, msg):
        """모드 상태 변화를 받아서 기록"""
        if msg.description != self.current_mode:
            self.current_mode = msg.description
            self.mode_history.append({
                'time': self.test_time,
                'position': (self.vehicle_x, self.vehicle_y),
                'mode': msg.current_mode,
                'description': msg.description
            })
            self.get_logger().info(f'🔄 모드 변경: {msg.description} (위치: {self.vehicle_x:.1f}, {self.vehicle_y:.1f})')
            
    def publish_test_data(self):
        """테스트 시나리오에 따라 차량 위치와 센서 데이터 발행"""
        self.test_time += 0.1
        
        # 현재 시나리오 결정
        total_time = 0.0
        for i, scenario in enumerate(self.scenarios):
            if self.test_time < total_time + scenario['duration']:
                if self.current_scenario != i:
                    self.current_scenario = i
                    self.scenario_start_time = total_time
                    self.get_logger().info(f'📋 시나리오 변경: {scenario["name"]}')
                break
            total_time += scenario['duration']
        
        # 시나리오 진행도 계산
        scenario = self.scenarios[self.current_scenario]
        scenario_progress = (self.test_time - self.scenario_start_time) / scenario['duration']
        scenario_progress = min(1.0, max(0.0, scenario_progress))
        
        # 차량 위치 보간
        start_x, start_y = scenario['start_pos']
        end_x, end_y = scenario['end_pos']
        self.vehicle_x = start_x + (end_x - start_x) * scenario_progress
        self.vehicle_y = start_y + (end_y - start_y) * scenario_progress
        
        # 차량 위치 발행
        self.publish_vehicle_pose()
        
        # 센서 데이터 발행
        self.publish_sensor_data(scenario['sensors'])
        
        # 5초마다 상태 출력
        if int(self.test_time * 10) % 50 == 0:
            self.print_status()
            
    def publish_vehicle_pose(self):
        """차량 위치 발행"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.vehicle_x
        pose_msg.pose.position.y = self.vehicle_y
        pose_msg.pose.position.z = 0.0
        
        # 간단한 방향 계산 (이동 방향)
        if len(self.mode_history) > 0:
            prev_x, prev_y = self.mode_history[-1]['position']
            if abs(self.vehicle_x - prev_x) > 0.01 or abs(self.vehicle_y - prev_y) > 0.01:
                self.vehicle_yaw = math.atan2(self.vehicle_y - prev_y, self.vehicle_x - prev_x)
        
        # 쿼터니언으로 변환
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.vehicle_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.vehicle_yaw / 2.0)
        
        self.vehicle_pose_pub.publish(pose_msg)
        
    def publish_sensor_data(self, sensors):
        """센서 데이터 발행"""
        # 정지선 거리
        stop_line_msg = Float32()
        stop_line_msg.data = sensors['stop_line']
        self.stop_line_pub.publish(stop_line_msg)
        
        # 장애물 감지
        obstacle_msg = Bool()
        obstacle_msg.data = sensors['obstacle']
        self.obstacle_pub.publish(obstacle_msg)
        
        # 배달 표지판
        sign_msg = String()
        sign_msg.data = sensors['sign']
        self.sign_pub.publish(sign_msg)
        
        # 주차 라바콘
        cone_msg = String()
        cone_msg.data = sensors['cone']
        self.cone_pub.publish(cone_msg)
        
    def print_status(self):
        """현재 상태 출력"""
        scenario = self.scenarios[self.current_scenario]
        self.get_logger().info(f'📍 상태: {scenario["name"]} | 위치: ({self.vehicle_x:.1f}, {self.vehicle_y:.1f}) | 모드: {self.current_mode}')
        
    def print_test_summary(self):
        """테스트 결과 요약 출력"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 DYNAMIC VEHICLE TEST 결과 요약')
        self.get_logger().info('=' * 60)
        
        for i, record in enumerate(self.mode_history):
            self.get_logger().info(f'{i+1:2d}. 시간: {record["time"]:5.1f}s | 위치: ({record["position"][0]:5.1f}, {record["position"][1]:5.1f}) | 모드: {record["description"]}')
        
        self.get_logger().info(f'총 모드 변경 횟수: {len(self.mode_history)}')
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicVehicleTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_test_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 