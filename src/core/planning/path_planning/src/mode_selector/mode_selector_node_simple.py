#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, UInt8
from enum import Enum, auto

class VehicleMode:
    DRIVE = 0
    OBSTACLE_STATIC = 1
    PAUSE = 2
    DELIVERY = 3
    PARKING = 4

class ModeSelectorSimple(Node):
    def __init__(self):
        super().__init__('mode_selector_simple')

        # Publisher (모드 발행)
        self.publisher_ = self.create_publisher(UInt8, '/mode_state', 10)
        self.desc_publisher_ = self.create_publisher(String, '/mode_description', 10)

        # Subscriber (센서 데이터 수신)
        self.create_subscription(Float32, '/stop_line_distance', self.stop_line_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/sign_detector', self.delivery_sign_callback, 10)
        self.create_subscription(String, '/cone_detector', self.parking_cone_callback, 10)

        # 타이머 설정 (1초마다 모드 판단 및 발행)
        self.timer = self.create_timer(1.0, self.publish_mode)

        # 초기값 설정
        self.stop_line_distance = 10.0  # 초기 충분히 먼 거리로 설정
        self.obstacle_detected = False  # 초기 장애물 없음
        self.delivery_sign_detected = False
        self.cone_parking_zone_detected = False

        # 미션 수행 플래그
        self.delivery_mission_completed = False
        self.parking_mission_completed = False

        self.state = VehicleMode.DRIVE

        self.get_logger().info('Mode Selector Simple 노드가 시작되었습니다.')
        self.get_logger().info(f'퍼블리셔 생성됨: /mode_state, /mode_description')

    # Callbacks
    def stop_line_callback(self, msg):
        self.stop_line_distance = msg.data
        self.get_logger().info(f'받은 정지선 거리: {self.stop_line_distance:.2f}m')

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        self.get_logger().info(f'받은 장애물 상태: {self.obstacle_detected}')

    def delivery_sign_callback(self, msg):
        self.delivery_sign_detected = True
        self.get_logger().info('배달 표지판이 감지되었습니다.')

    def parking_cone_callback(self, msg):
        self.cone_parking_zone_detected = True
        self.get_logger().info('주차 구역 (라바콘)이 감지되었습니다.')
        

    # FSM 상태 업데이트
    def update_mode(self):
        previous_state = self.state
        mode_msg = UInt8()
        desc_msg = String()

        # 1. 배달 미션 중이면 계속 유지
        if self.state == VehicleMode.DELIVERY:
            mode_msg.data = VehicleMode.DELIVERY
            desc_msg.data = 'Delivery Mode: 배달 미션 수행 중'
            if self.delivery_mission_completed:
                self.get_logger().info("[FSM] 배달 미션 완료됨. DRIVE 모드로 전환.")
                self.state = VehicleMode.DRIVE
                self.delivery_sign_detected = False  # 초기화

        # 2. 주차 미션 중이면 계속 유지
        elif self.state == VehicleMode.PARKING:
            mode_msg.data = VehicleMode.PARKING
            desc_msg.data = 'Parking Mode: 주차 미션 수행 중'
            if self.parking_mission_completed:
                self.get_logger().info("[FSM] 주차 미션 완료됨. DRIVE 모드로 전환.")
                self.state = VehicleMode.DRIVE
                self.cone_parking_zone_detected = False  # 초기화

        # 3. 새 배달 미션 감지 (한 번만 진입)
        elif self.delivery_sign_detected and not self.delivery_mission_completed:
            self.state = VehicleMode.DELIVERY
            mode_msg.data = VehicleMode.DELIVERY
            desc_msg.data = 'Delivery Mode: 배달 미션 시작'

        # 4. 새 주차 미션 감지 (한 번만 진입)
        elif self.cone_parking_zone_detected and not self.parking_mission_completed:
            self.state = VehicleMode.PARKING
            mode_msg.data = VehicleMode.PARKING
            desc_msg.data = 'Parking Mode: 주차 미션 시작'

        # 5. 정적 장애물
        elif self.obstacle_detected:
            mode_msg.data = VehicleMode.OBSTACLE_STATIC
            desc_msg.data = 'Obstacle Detected: 정적 장애물 회피 모드'

        # 6. 정지선 감지
        elif self.stop_line_distance < 3.0:
            mode_msg.data = VehicleMode.PAUSE
            desc_msg.data = 'Pause Mode: 정지선 가까움'

        # 7. 기본 주행 모드
        else:
            mode_msg.data = VehicleMode.DRIVE
            desc_msg.data = 'Drive Mode: 정상 주행'

        # 상태 전이가 발생했을 때만 publish
        if previous_state != self.state or self.state in (VehicleMode.DELIVERY, VehicleMode.PARKING):
            self.get_logger().info(f'[FSM] Mode Changed: {desc_msg.data}')

    # 상태 발행
    def publish_mode(self):
        # FSM 상태 업데이트 먼저 수행
        self.update_mode()
        
        # 현재 상태를 발행 (항상 발행)
        mode_msg = UInt8()
        desc_msg = String()
        
        mode_msg.data = self.state
        if self.state == VehicleMode.DRIVE:
            desc_msg.data = 'Drive Mode: 정상 주행'
        elif self.state == VehicleMode.OBSTACLE_STATIC:
            desc_msg.data = 'Obstacle Detected: 정적 장애물 회피 모드'
        elif self.state == VehicleMode.PAUSE:
            desc_msg.data = 'Pause Mode: 정지선 가까움'
        elif self.state == VehicleMode.DELIVERY:
            desc_msg.data = 'Delivery Mode: 배달 미션 수행 중'
        elif self.state == VehicleMode.PARKING:
            desc_msg.data = 'Parking Mode: 주차 미션 수행 중'
        else:
            desc_msg.data = 'Unknown Mode'
        
        self.publisher_.publish(mode_msg)
        self.desc_publisher_.publish(desc_msg)
        self.get_logger().info(f'[FSM] Current Mode: {desc_msg.data} (State: {self.state}) - 토픽 발행됨')

def main(args=None):
    rclpy.init(args=args)
    mode_selector_node = ModeSelectorSimple()
    rclpy.spin(mode_selector_node)
    mode_selector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 