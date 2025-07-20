#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from planning_msgs.msg import ModeState
import time
import math

class ModeSelectorTest(Node):
    def __init__(self):
        super().__init__('mode_selector_test')
        
        # Publishers for sensor data simulation
        self.stop_line_pub = self.create_publisher(Float32, '/stop_line_distance', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.sign_pub = self.create_publisher(String, '/sign_detector', 10)
        self.cone_pub = self.create_publisher(String, '/cone_detector', 10)
        
        # Subscriber for mode state
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_callback, 10)
        
        # Timer for publishing test data
        self.timer = self.create_timer(0.5, self.publish_test_data)
        
        # Test scenario variables
        self.test_time = 0.0
        self.current_scenario = 0
        self.scenarios = [
            {'name': 'Normal Driving', 'duration': 5.0},
            {'name': 'Stop Line Detection', 'duration': 3.0},
            {'name': 'Obstacle Detection', 'duration': 3.0},
            {'name': 'Delivery Sign Detection', 'duration': 5.0},
            {'name': 'Parking Cone Detection', 'duration': 5.0},
            {'name': 'Return to Normal', 'duration': 3.0}
        ]
        
        # Current mode tracking
        self.current_mode = "UNKNOWN"
        self.mode_history = []
        
        self.get_logger().info('Mode Selector Test Node 시작됨')
        
    def mode_callback(self, msg):
        """모드 상태 변화를 받아서 기록"""
        if msg.description != self.current_mode:
            self.current_mode = msg.description
            self.mode_history.append({
                'time': self.test_time,
                'mode': msg.current_mode,
                'description': msg.description
            })
            self.get_logger().info(f'🔄 모드 변경: {msg.description}')
            
    def publish_test_data(self):
        """테스트 시나리오에 따라 센서 데이터 발행"""
        self.test_time += 0.5
        
        # 현재 시나리오 결정
        scenario_start_time = 0.0
        for i, scenario in enumerate(self.scenarios):
            if self.test_time < scenario_start_time + scenario['duration']:
                self.current_scenario = i
                break
            scenario_start_time += scenario['duration']
        
        # 시나리오별 데이터 발행
        if self.current_scenario == 0:  # Normal Driving
            self.publish_normal_driving()
        elif self.current_scenario == 1:  # Stop Line Detection
            self.publish_stop_line_detection()
        elif self.current_scenario == 2:  # Obstacle Detection
            self.publish_obstacle_detection()
        elif self.current_scenario == 3:  # Delivery Sign Detection
            self.publish_delivery_sign_detection()
        elif self.current_scenario == 4:  # Parking Cone Detection
            self.publish_parking_cone_detection()
        elif self.current_scenario == 5:  # Return to Normal
            self.publish_normal_driving()
            
        # 시나리오 정보 출력
        if int(self.test_time * 2) % 10 == 0:  # 5초마다 출력
            self.print_scenario_info()
            
    def publish_normal_driving(self):
        """정상 주행 시나리오"""
        # 정지선 거리: 충분히 먼 거리
        stop_line_msg = Float32()
        stop_line_msg.data = 50.0
        self.stop_line_pub.publish(stop_line_msg)
        
        # 장애물: 없음
        obstacle_msg = Bool()
        obstacle_msg.data = False
        self.obstacle_pub.publish(obstacle_msg)
        
        # 표지판: 없음
        sign_msg = String()
        sign_msg.data = ""
        self.sign_pub.publish(sign_msg)
        
        # 라바콘: 없음
        cone_msg = String()
        cone_msg.data = ""
        self.cone_pub.publish(cone_msg)
        
    def publish_stop_line_detection(self):
        """정지선 감지 시나리오"""
        # 정지선 거리: 점진적으로 가까워짐
        stop_line_msg = Float32()
        stop_line_msg.data = max(0.5, 3.0 - (self.test_time - 5.0))
        self.stop_line_pub.publish(stop_line_msg)
        
        # 나머지는 정상 주행과 동일
        obstacle_msg = Bool()
        obstacle_msg.data = False
        self.obstacle_pub.publish(obstacle_msg)
        
        sign_msg = String()
        sign_msg.data = ""
        self.sign_pub.publish(sign_msg)
        
        cone_msg = String()
        cone_msg.data = ""
        self.cone_pub.publish(cone_msg)
        
    def publish_obstacle_detection(self):
        """장애물 감지 시나리오"""
        # 정지선 거리: 충분히 먼 거리
        stop_line_msg = Float32()
        stop_line_msg.data = 50.0
        self.stop_line_pub.publish(stop_line_msg)
        
        # 장애물: 감지됨
        obstacle_msg = Bool()
        obstacle_msg.data = True
        self.obstacle_pub.publish(obstacle_msg)
        
        # 나머지는 정상 주행과 동일
        sign_msg = String()
        sign_msg.data = ""
        self.sign_pub.publish(sign_msg)
        
        cone_msg = String()
        cone_msg.data = ""
        self.cone_pub.publish(cone_msg)
        
    def publish_delivery_sign_detection(self):
        """배달 표지판 감지 시나리오"""
        # 정지선 거리: 충분히 먼 거리
        stop_line_msg = Float32()
        stop_line_msg.data = 50.0
        self.stop_line_pub.publish(stop_line_msg)
        
        # 장애물: 없음
        obstacle_msg = Bool()
        obstacle_msg.data = False
        self.obstacle_pub.publish(obstacle_msg)
        
        # 배달 표지판: 감지됨
        sign_msg = String()
        sign_msg.data = "DELIVERY"
        self.sign_pub.publish(sign_msg)
        
        # 라바콘: 없음
        cone_msg = String()
        cone_msg.data = ""
        self.cone_pub.publish(cone_msg)
        
    def publish_parking_cone_detection(self):
        """주차 라바콘 감지 시나리오"""
        # 정지선 거리: 충분히 먼 거리
        stop_line_msg = Float32()
        stop_line_msg.data = 50.0
        self.stop_line_pub.publish(stop_line_msg)
        
        # 장애물: 없음
        obstacle_msg = Bool()
        obstacle_msg.data = False
        self.obstacle_pub.publish(obstacle_msg)
        
        # 배달 표지판: 없음
        sign_msg = String()
        sign_msg.data = ""
        self.sign_pub.publish(sign_msg)
        
        # 주차 라바콘: 감지됨
        cone_msg = String()
        cone_msg.data = "PARKING"
        self.cone_pub.publish(cone_msg)
        
    def print_scenario_info(self):
        """현재 시나리오 정보 출력"""
        scenario = self.scenarios[self.current_scenario]
        self.get_logger().info(f'📋 현재 시나리오: {scenario["name"]} (시간: {self.test_time:.1f}s)')
        
    def print_test_summary(self):
        """테스트 결과 요약 출력"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('📊 MODE SELECTOR 테스트 결과 요약')
        self.get_logger().info('=' * 50)
        
        for i, history in enumerate(self.mode_history):
            self.get_logger().info(f'{i+1:2d}. {history["time"]:5.1f}s: {history["description"]}')
            
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'총 모드 변경 횟수: {len(self.mode_history)}')

def main(args=None):
    rclpy.init(args=args)
    test_node = ModeSelectorTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.print_test_summary()
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 