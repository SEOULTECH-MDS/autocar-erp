#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import math
import threading
import sys
import select
import termios
import tty
from autocar_utils.euler_from_quaternion import euler_from_quaternion
from autocar_utils.yaw_to_quaternion import yaw_to_quaternion
from geometry_msgs.msg import Quaternion, Pose, PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
from pyproj import Proj, Transformer

# GPS, IMU 데이터 시뮬레이션 노드
class SimulationPub(Node):
    def __init__(self):
        super().__init__('simulation_pub')
        
        # 퍼블리셔 설정
        self.publisher_gps = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)

        # self.obstacle_marker_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self.obstacle_pose_pub = self.create_publisher(PoseArray, '/sensor_fusion/obstacle', 10)
        self.stopline_marker_pub = self.create_publisher(MarkerArray, '/stoplines/markers', 10)
        # 장애물 정보 설정
        # self.obstacle_lat = 37.62999526
        # self.obstacle_lon = 127.08136022
        # self.obstacle_a = 0.5  # 장축 길이 (meter)
        # self.obstacle_b = 0.5  # 단축 길이 (meter)
        
        # self.obstacle1_lat = 37.63004349
        # self.obstacle1_lon = 127.08137807
        # self.obstacle2_lat = 37.62999185
        # self.obstacle2_lon = 127.08135382

        self.obstacle1_lat = 37.62998188
        self.obstacle1_lon = 127.08135814
        self.obstacle2_lat = 37.62995006
        self.obstacle2_lon = 127.08134290
        
        # 정지선 정보 설정
        self.stopline_lat = 37.62983145
        self.stopline_lon = 127.08144689
        
        # UTM 변환 설정 (한국 지역에 맞는 설정)
        self.transformer = Transformer.from_proj(
            Proj(proj='latlong', ellps='WGS84', datum='WGS84'),
            Proj(proj='utm', zone=52, ellps='WGS84'),
            always_xy=True
        )
        # 장애물의 UTM 좌표 계산
        self.obstacle1_x, self.obstacle1_y = self.transformer.transform(self.obstacle1_lon, self.obstacle1_lat)
        self.obstacle2_x, self.obstacle2_y = self.transformer.transform(self.obstacle2_lon, self.obstacle2_lat)
        
        # 정지선의 UTM 좌표 계산
        self.stopline_x, self.stopline_y = self.transformer.transform(self.stopline_lon, self.stopline_lat)
        
        # 장애물 리스트 생성
        self.obstacles = [
            {
                'x': self.obstacle1_x,
                'y': self.obstacle1_y,
                'z' : 0.0,
                'label': 0
            },
            {
                'x': self.obstacle2_x,
                'y': self.obstacle2_y,
                'z': 0.0,
                'label': 0
            }
        ]

        # 정지선 정보 생성
        self.stoplines = [
            {
                'x': self.stopline_x,
                'y': self.stopline_y,
                'color': ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9),  # 흰색
                'scale': {'x': 4.0, 'y': 0.15, 'z': 0.05},  # 정지선 형태 (길고 얇음)
                'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': np.deg2rad(-90.0)}  # 도로에 수직 (차량 초기 방향에 수직)
            }
        ]

        self.get_logger().info("SimulationPub START \
                               \n 모드 전환 키: m \
                               \n 수동 모드: w(전진), s(후진), a(좌회전), d(우회전) \
                               \n 종료 키: q")

        # 조작 모드 설정
        self.is_manual_mode = False  # False: 자동모드, True: 수동모드
        self.manual_speed = 2.0  # 수동 모드 기본 속도 (m/s)
        self.manual_turn_rate = 0.5  # 수동 모드 회전 속도 (rad/s)
        
        # 로그 제어 변수
        self.last_mode_logged = None  # 마지막으로 로그에 출력한 모드

        # 초기 차량 상태 파라미터 설정
        self.declare_parameter('initial_latitude', 37.630096)  # 미래관 주차장
        self.declare_parameter('initial_longitude', 127.081397)
        # self.declare_parameter('initial_latitude', 37.239205)  # KCITY
        # self.declare_parameter('initial_longitude', 126.773193)
        self.declare_parameter('initial_yaw_deg', -70.0)
        self.declare_parameter('wheel_base', 1.566)  # 차량 휠베이스 (m)

        # 파라미터 값 가져오기
        self.latitude = self.get_parameter('initial_latitude').get_parameter_value().double_value
        self.longitude = self.get_parameter('initial_longitude').get_parameter_value().double_value
        self.yaw = np.deg2rad(self.get_parameter('initial_yaw_deg').get_parameter_value().double_value)
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        # 차량 상태 변수
        self.velocity = 0.0  # 현재 속도 (m/s)
        self.steering_angle = 0.0  # 현재 조향각 (라디안)
        
        # UTM 좌표로 변환
        self.x, self.y = self.transformer.transform(self.longitude, self.latitude)
        
        # 사용자 입력 구독자
        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped,
            '/erp/cmd_vel',
            self.cmd_callback,
            10
        )

        # 타이머 설정
        self.dt = 0.05  # 20Hz
        self.sensor_rate = 0.1
        self.create_timer(self.sensor_rate, self.publish_data)
        
        # 키보드 입력 스레드 시작
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()
    

    def keyboard_listener(self):
        """키보드 입력을 실시간으로 감지하는 함수"""
        # 터미널 설정 저장
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():  # ROS2가 실행 중일 때만 루프
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    key = sys.stdin.read(1)
                    self.handle_key_input(key)
                    if key.lower() == 'q':
                        self.get_logger().info("종료 키 입력됨")
                        rclpy.shutdown()
                        break
        except Exception as e:
            self.get_logger().error(f"키보드 리스너 오류: {e}")
        finally:
            # 터미널 설정 복원
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key_input(self, key):
        """키 입력 처리"""
        if key.lower() == 'm':
            self.is_manual_mode = not self.is_manual_mode
            mode_str = "수동" if self.is_manual_mode else "자동"
            self.get_logger().info(f"모드 전환: {mode_str} 모드")
        
        elif self.is_manual_mode:
            # 수동 모드에서만 동작
            if key.lower() == 'w':
                self.manual_move_forward()
            elif key.lower() == 's':
                self.manual_move_backward()
            elif key.lower() == 'a':
                self.manual_turn_left()
            elif key.lower() == 'd':
                self.manual_turn_right()

    def manual_move_forward(self):
        """수동 모드: 전진"""
        self.x += self.manual_speed * math.cos(self.yaw) * self.dt * 10  # 10배 빠르게
        self.y += self.manual_speed * math.sin(self.yaw) * self.dt * 10
        self.longitude, self.latitude = self.transformer.transform(self.x, self.y, direction='INVERSE')
        self.velocity = self.manual_speed  # 수동 모드에서 속도 표시용

    def manual_move_backward(self):
        """수동 모드: 후진"""
        self.x -= self.manual_speed * math.cos(self.yaw) * self.dt * 10
        self.y -= self.manual_speed * math.sin(self.yaw) * self.dt * 10
        self.longitude, self.latitude = self.transformer.transform(self.x, self.y, direction='INVERSE')
        self.velocity = -self.manual_speed  # 수동 모드에서 속도 표시용 (후진)

    def manual_turn_left(self):
        """수동 모드: 좌회전"""
        self.yaw += self.manual_turn_rate * self.dt * 10
        self.yaw = self.normalize_angle(self.yaw)

    def manual_turn_right(self):
        """수동 모드: 우회전"""
        self.yaw -= self.manual_turn_rate * self.dt * 10
        self.yaw = self.normalize_angle(self.yaw)

    # 사용자 입력(속도, 조향각) 콜백 함수
    def cmd_callback(self, msg):
        # 자동 모드에서만 사용자 입력 적용
        if not self.is_manual_mode:
            self.velocity = msg.drive.speed  # m/s
            self.steering_angle = msg.drive.steering_angle  # 라디안
            self.update_vehicle_state()
        # 수동 모드에서는 사용자 입력 무시
    
    # 차량 상태 업데이트 (자전거 모델)
    def update_vehicle_state(self):
        # if abs(self.velocity) < 0.001:  # 정지 상태면 업데이트 안함
        #     return
            
        # 자전거 모델 (비선형)에 따른 위치 및 방향 업데이트
        self.yaw += self.velocity / self.wheel_base * math.tan(self.steering_angle) * self.dt
        self.yaw = self.normalize_angle(self.yaw)
        
        # UTM 좌표 업데이트
        self.x += self.velocity * math.cos(self.yaw) * self.dt
        self.y += self.velocity * math.sin(self.yaw) * self.dt
        
        # UTM 좌표를 위도/경도로 변환
        self.longitude, self.latitude = self.transformer.transform(self.x, self.y, direction='INVERSE')

    # 각도 정규화 (-pi ~ pi)
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # GPS 및 IMU 데이터 퍼블리싱
    def publish_data(self):
        # GPS 메시지 퍼블리싱
        gps_msg = NavSatFix()
        gps_msg.latitude = self.latitude
        gps_msg.longitude = self.longitude
        self.publisher_gps.publish(gps_msg)
        
        # IMU 메시지 퍼블리싱
        imu_msg = Imu()
        q = yaw_to_quaternion(self.yaw)
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        
        # 각속도 계산 (yaw 축 회전)
        if abs(self.velocity) > 0.001:
            imu_msg.angular_velocity.z = self.velocity / self.wheel_base * math.tan(self.steering_angle)
        else:
            imu_msg.angular_velocity.z = 0.0
            
        self.publisher_imu.publish(imu_msg)
        # self.publish_obstacle_marker()
        self.publish_obstacle_posearray()
        self.publish_stopline_marker()

        # 모드 변경 시에만 로그 출력
        current_mode = "수동" if self.is_manual_mode else "자동"
        if self.last_mode_logged != current_mode:
            self.get_logger().info(f"현재 모드: {current_mode}")
            self.last_mode_logged = current_mode



    # def publish_obstacle_marker(self):
    #     marker_array = MarkerArray()
        
    #     # for문으로 장애물 마커들 생성
    #     for i, obstacle in enumerate(self.obstacles):
    #         marker = Marker()
    #         marker.header.frame_id = "world"
    #         marker.header.stamp = self.get_clock().now().to_msg()
    #         marker.ns = "obstacles"
    #         marker.id = i
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
            
    #         # 위치 설정
    #         marker.pose.position.x = obstacle['x']
    #         marker.pose.position.y = obstacle['y']
    #         marker.pose.position.z = 0.0
            
    #         # 크기 설정
    #         marker.scale.x = obstacle['scale']['x']
    #         marker.scale.y = obstacle['scale']['y']
    #         marker.scale.z = obstacle['scale']['z']
            
    #         # 색상 설정
    #         marker.color = obstacle['color']
            
    #         marker_array.markers.append(marker)
        
    #     self.obstacle_marker_pub.publish(marker_array)

    def publish_obstacle_posearray(self):
        """장애물을 PoseArray로 퍼블리시"""
        pose_array = PoseArray()
        pose_array.header.frame_id = "world"  # 또는 "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        # 각 장애물을 Pose로 변환
        for obstacle in self.obstacles:
            pose = Pose()
            
            # 위치 설정
            pose.position.x = float(obstacle['x'])
            pose.position.y = float(obstacle['y'])
            pose.position.z = float(obstacle['z'])
            
            # 방향 설정 (기본값: 단위 쿼터니언)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            # 라벨 정보는 orientation.z에 저장 (다른 곳에서 이렇게 사용하는 것으로 보임)
            # pose.orientation.z = float(obstacle['label'])  # 필요시 사용
            
            pose_array.poses.append(pose)
        
        self.obstacle_pose_pub.publish(pose_array)
        # 로그에 장애물 정보 추가 (로그 출력 제거)
        # 장애물 마커만 발행하고 로그는 출력하지 않음

    def publish_stopline_marker(self):
        """정지선 마커 퍼블리시"""
        marker_array = MarkerArray()
        
        # 정지선 마커들 생성
        for i, stopline in enumerate(self.stoplines):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "stoplines"
            marker.id = i
            marker.type = Marker.CUBE  # 정지선은 직육면체로 표현
            marker.action = Marker.ADD
            
            # 위치 설정
            marker.pose.position.x = stopline['x']
            marker.pose.position.y = stopline['y']
            marker.pose.position.z = 0.05  # 지면보다 약간 위에
            
            # 방향 설정 (도로 방향에 맞게)
            orientation = stopline['orientation']
            q = yaw_to_quaternion(orientation['yaw'])
            marker.pose.orientation.x = q.x
            marker.pose.orientation.y = q.y
            marker.pose.orientation.z = q.z
            marker.pose.orientation.w = q.w
            
            # 크기 설정 (정지선 형태)
            marker.scale.x = stopline['scale']['x']  # 길이
            marker.scale.y = stopline['scale']['y']  # 폭 (얇게)
            marker.scale.z = stopline['scale']['z']  # 높이
            
            # 색상 설정 (흰색 정지선)
            marker.color = stopline['color']
            
            marker_array.markers.append(marker)
        
        self.stopline_marker_pub.publish(marker_array)
        


def main(args=None):
    rclpy.init(args=args)
    simulation_pub = SimulationPub()
    rclpy.spin(simulation_pub)
    simulation_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 