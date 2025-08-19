#!/usr/bin/env python3
"""
Simple Test Node - 센서 데이터 시뮬레이션 노드

이 파일은 모드 셀렉터의 동작을 테스트하기 위한 센서 데이터를 시뮬레이션합니다.

주요 기능:
- 정지선 거리, 장애물 감지, 배달 표지판, 주차 라바콘 데이터 시뮬레이션
- 미션 시나리오를 순환하며 반복 실행
- 모드 셀렉터의 상태 변화를 모니터링하고 기록
- 테스트 결과 요약 제공

발행 토픽:
- /stop_line_distance (Float32): 정지선까지의 거리
- /obstacle_detected (Bool): 장애물 감지 여부
- /sign_detector (String): 배달 표지판 감지
- /cone_detector (String): 주차 라바콘 감지

구독 토픽:
- /mode_state (UInt8): 현재 모드 상태
- /mode_description (String): 모드 설명

사용법:
    ros2 run path_planning simple_test
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, UInt8
import time
import math

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        
        # Publishers for sensor data simulation
        self.stop_line_pub = self.create_publisher(Float32, '/stop_line_distance', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.sign_pub = self.create_publisher(String, '/sign_detector', 10)
        self.cone_pub = self.create_publisher(String, '/cone_detector', 10)
        
        # Subscriber for mode state
        self.mode_sub = self.create_subscription(UInt8, '/mode_state', self.mode_callback, 10)
        self.desc_sub = self.create_subscription(String, '/mode_description', self.desc_callback, 10)
        
        # Timer for publishing test data
        self.timer = self.create_timer(0.5, self.publish_test_data)
        
        # Test scenario variables
        self.test_time = 0.0
        self.current_scenario = 0
        self.scenarios = [
            {'name': '정상 주행', 'duration': 5.0, 'stop_line': 50.0, 'obstacle': False, 'sign': '', 'cone': ''},
            {'name': '정지선 감지', 'duration': 3.0, 'stop_line': 2.0, 'obstacle': False, 'sign': '', 'cone': ''},
            {'name': '장애물 감지', 'duration': 3.0, 'stop_line': 50.0, 'obstacle': True, 'sign': '', 'cone': ''},
            {'name': '배달 표지판', 'duration': 5.0, 'stop_line': 50.0, 'obstacle': False, 'sign': 'DELIVERY', 'cone': ''},
            {'name': '주차 라바콘', 'duration': 5.0, 'stop_line': 50.0, 'obstacle': False, 'sign': '', 'cone': 'PARKING'},
            {'name': '정상 복귀', 'duration': 3.0, 'stop_line': 50.0, 'obstacle': False, 'sign': '', 'cone': ''}
        ]
        
        # Current mode tracking
        self.current_mode = 0
        self.current_description = "UNKNOWN"
        self.mode_history = []
        
        self.get_logger().info('🚗 Simple Test Node 시작됨')
        
    def mode_callback(self, msg):
        """모드 상태 변화를 받아서 기록"""
        if msg.data != self.current_mode:
            self.current_mode = msg.data
            self.mode_history.append({
                'time': self.test_time,
                'mode': msg.data,
                'description': self.current_description
            })
            self.get_logger().info(f'🔄 모드 변경: {self.current_description} (모드: {msg.data})')
            
    def desc_callback(self, msg):
        """모드 설명을 받아서 저장"""
        self.current_description = msg.data
            
    def publish_test_data(self):
        """테스트 시나리오에 따라 센서 데이터 발행"""
        self.test_time += 0.5
        
        # 전체 미션 시간 계산
        total_mission_time = sum(scenario['duration'] for scenario in self.scenarios)
        
        # 미션 반복을 위한 시간 조정
        cycle_time = self.test_time % total_mission_time
        
        # 현재 시나리오 결정
        scenario_start_time = 0.0
        for i, scenario in enumerate(self.scenarios):
            if cycle_time < scenario_start_time + scenario['duration']:
                if self.current_scenario != i:
                    self.current_scenario = i
                    self.get_logger().info(f'📋 시나리오 변경: {scenario["name"]} (사이클: {int(self.test_time // total_mission_time) + 1})')
                break
            scenario_start_time += scenario['duration']
        
        # 현재 시나리오 데이터 발행
        scenario = self.scenarios[self.current_scenario]
        
        # 정지선 거리
        stop_line_msg = Float32()
        stop_line_msg.data = scenario['stop_line']
        self.stop_line_pub.publish(stop_line_msg)
        
        # 장애물 감지
        obstacle_msg = Bool()
        obstacle_msg.data = scenario['obstacle']
        self.obstacle_pub.publish(obstacle_msg)
        
        # 배달 표지판
        sign_msg = String()
        sign_msg.data = scenario['sign']
        self.sign_pub.publish(sign_msg)
        
        # 주차 라바콘
        cone_msg = String()
        cone_msg.data = scenario['cone']
        self.cone_pub.publish(cone_msg)
        
        # 5초마다 상태 출력
        if int(self.test_time * 2) % 10 == 0:
            self.print_status()
            
    def print_status(self):
        """현재 상태 출력"""
        scenario = self.scenarios[self.current_scenario]
        total_mission_time = sum(scenario['duration'] for scenario in self.scenarios)
        cycle_num = int(self.test_time // total_mission_time) + 1
        cycle_time = self.test_time % total_mission_time
        self.get_logger().info(f'📍 상태: {scenario["name"]} | 사이클: {cycle_num} | 시간: {cycle_time:.1f}s | 모드: {self.current_description}')
        
    def print_test_summary(self):
        """테스트 결과 요약 출력"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('📊 SIMPLE TEST 결과 요약')
        self.get_logger().info('=' * 50)
        
        for i, record in enumerate(self.mode_history):
            self.get_logger().info(f'{i+1:2d}. 시간: {record["time"]:5.1f}s | 모드: {record["mode"]} | 설명: {record["description"]}')
        
        self.get_logger().info(f'총 모드 변경 횟수: {len(self.mode_history)}')
        self.get_logger().info('=' * 50)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_test_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 