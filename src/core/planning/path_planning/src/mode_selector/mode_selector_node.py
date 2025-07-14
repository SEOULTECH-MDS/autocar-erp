#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from planning_msgs.msg import ModeState
from std_msgs.msg import Float32, Bool
from enum import Enum, auto
from std_msgs.msg import String
class VehicleMode:
    DRIVE = 0
    OBSTACLE_STATIC = 1
    PAUSE = 2
    DELIVERY = 3
    PARKING = 4

class ModeSelector(Node):
    def __init__(self):
        super().__init__('mode_selector')

        # Publisher (모드 발행)
        self.publisher_ = self.create_publisher(ModeState, '/mode_state', 10)

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

        self.get_logger().info('Mode Selector 노드가 시작되었습니다.')

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
        mode_msg = ModeState()

        # 1. 배달 미션 중이면 계속 유지
        if self.state == VehicleMode.DELIVERY:
            mode_msg.current_mode = ModeState.DELIVERY
            mode_msg.description = 'Delivery Mode: 배달 미션 수행 중'
            if self.delivery_mission_completed:
                self.get_logger().info("[FSM] 배달 미션 완료됨. DRIVE 모드로 전환.")
                self.state = VehicleMode.DRIVE
                self.delivery_sign_detected = False  # 초기화

        # 2. 주차 미션 중이면 계속 유지
        elif self.state == VehicleMode.PARKING:
            mode_msg.current_mode = ModeState.PARKING
            mode_msg.description = 'Parking Mode: 주차 미션 수행 중'
            if self.parking_mission_completed:
                self.get_logger().info("[FSM] 주차 미션 완료됨. DRIVE 모드로 전환.")
                self.state = VehicleMode.DRIVE
                self.cone_parking_zone_detected = False  # 초기화

        # 3. 새 배달 미션 감지 (한 번만 진입)
        elif self.delivery_sign_detected and not self.delivery_mission_completed:
            self.state = VehicleMode.DELIVERY
            mode_msg.current_mode = ModeState.DELIVERY
            mode_msg.description = 'Delivery Mode: 배달 미션 시작'

        # 4. 새 주차 미션 감지 (한 번만 진입)
        elif self.cone_parking_zone_detected and not self.parking_mission_completed:
            self.state = VehicleMode.PARKING
            mode_msg.current_mode = ModeState.PARKING
            mode_msg.description = 'Parking Mode: 주차 미션 시작'

        # 5. 정적 장애물
        elif self.obstacle_detected:
            mode_msg.current_mode = ModeState.OBSTACLE_STATIC
            mode_msg.description = 'Obstacle Detected: 정적 장애물 회피 모드'

        # 6. 정지선 감지
        elif self.stop_line_distance < 3.0:
            mode_msg.current_mode = ModeState.PAUSE
            mode_msg.description = 'Pause Mode: 정지선 가까움'

        # 7. 기본 주행 모드
        else:
            mode_msg.current_mode = ModeState.DRIVE
            mode_msg.description = 'Drive Mode: 정상 주행'

        # 상태 전이가 발생했을 때만 publish
        if previous_state != self.state or self.state in (VehicleMode.DELIVERY, VehicleMode.PARKING):
            self.publisher_.publish(mode_msg)
            self.get_logger().info(f'[FSM] Mode Changed: {mode_msg.description}')

            # 상태 변경 시에만 발행
            if previous_state != self.state:
                self.publish_mode()

    # 상태 발행
    def publish_mode(self):
        mode_msg = ModeState()

        if self.state == VehicleMode.DELIVERY:
            if self.delivery_mission_completed:
                self.state = VehicleMode.DRIVE
            mode_msg.current_mode = ModeState.DELIVERY
            mode_msg.description = '배달 모드 수행 중'

        elif self.state == VehicleMode.PARKING:
            if self.parking_mission_completed:
                self.state = VehicleMode.DRIVE
            mode_msg.current_mode = ModeState.PARKING
            mode_msg.description = '주차 모드 수행 중'

        elif self.obstacle_detected:
            mode_msg.current_mode = ModeState.OBSTACLE_STATIC
            mode_msg.description = 'Obstacle Detected: 정적 장애물 회피 모드'

        elif self.stop_line_distance < 3.0:
            mode_msg.current_mode = ModeState.PAUSE
            mode_msg.description = 'Pause Mode: 정지선 가까움'

        elif self.delivery_sign_detected and not self.delivery_mission_completed:
            self.state = VehicleMode.DELIVERY
            mode_msg.current_mode = ModeState.DELIVERY
            mode_msg.description = 'Delivery Mode: 배달 모드 시작'

        elif self.cone_parking_zone_detected and not self.parking_mission_completed:
            self.state = VehicleMode.PARKING
            mode_msg.current_mode = ModeState.PARKING
            mode_msg.description = 'Parking Mode: 주차 모드 시작'

        else:
            mode_msg.current_mode = ModeState.DRIVE
            mode_msg.description = 'Drive Mode: 정상 주행'

        self.publisher_.publish(mode_msg)
        self.get_logger().info(f'[FSM] Mode Changed: {mode_msg.description}')


        # Mode 메시지 발행
        self.publisher_.publish(mode_msg)
        self.get_logger().info(f'발행된 모드: {mode_msg.description}')

def main(args=None):
    rclpy.init(args=args)
    mode_selector_node = ModeSelector()
    rclpy.spin(mode_selector_node)
    mode_selector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
