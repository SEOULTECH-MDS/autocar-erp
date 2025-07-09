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
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from pyproj import Proj, Transformer

# GPS, IMU 데이터 시뮬레이션 노드
class SimulationPub(Node):
    def __init__(self):
        super().__init__('simulation_pub')
        
        # 퍼블리셔 설정
        self.publisher_gps = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)

        self.obstacle_marker_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        # 장애물 정보 설정
        self.obstacle_lat = 37.62999526
        self.obstacle_lon = 127.08136022
        self.obstacle_a = 0.5  # 장축 길이 (meter)
        self.obstacle_b = 0.5  # 단축 길이 (meter)


        # UTM 변환 설정 (한국 지역에 맞는 설정)
        self.transformer = Transformer.from_proj(
            Proj(proj='latlong', ellps='WGS84', datum='WGS84'),
            Proj(proj='utm', zone=52, ellps='WGS84'),
            always_xy=True
        )
        # 장애물의 UTM 좌표 계산
        self.obstacle_x, self.obstacle_y = self.transformer.transform(
            self.obstacle_lon, 
            self.obstacle_lat
        )

        self.get_logger().info("HitechSimulationPub START")

        # 초기 차량 상태 파라미터 설정
        self.declare_parameter('initial_latitude', 37.630096)  # 미래관 주차장
        self.declare_parameter('initial_longitude', 127.081397)
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
        self.dt = 0.1  # 10Hz
        self.create_timer(self.dt, self.publish_data)

    # 사용자 입력(속도, 조향각) 콜백 함수
    def cmd_callback(self, msg):
        self.velocity = msg.drive.speed  # m/s
        self.steering_angle = msg.drive.steering_angle  # 라디안
        self.update_vehicle_state()
    
    # 차량 상태 업데이트 (자전거 모델)
    def update_vehicle_state(self):
        if abs(self.velocity) < 0.001:  # 정지 상태면 업데이트 안함
            return
            
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
        imu_msg.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        
        # 각속도 계산 (yaw 축 회전)
        if abs(self.velocity) > 0.001:
            imu_msg.angular_velocity.z = self.velocity / self.wheel_base * math.tan(self.steering_angle)
        else:
            imu_msg.angular_velocity.z = 0.0
            
        self.publisher_imu.publish(imu_msg)
        
        # 로그 출력
        self.get_logger().info(f"\n lat={self.latitude:.8f}, lon={self.longitude:.8f}, \n Heading: {np.rad2deg(self.yaw):.2f} deg, Speed: {self.velocity:.2f} m/s")

        self.publish_obstacle_marker()

    def publish_obstacle_marker(self):
        marker_array = MarkerArray()
        
        # 장애물 마커 생성
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 위치 설정
        marker.pose.position.x = self.obstacle_x
        marker.pose.position.y = self.obstacle_y
        marker.pose.position.z = 0.0
        
        # 크기 설정 (장축과 단축 반영)
        marker.scale.x = self.obstacle_a * 2
        marker.scale.y = self.obstacle_b * 2
        marker.scale.z = 0.5
        
        # 색상 설정 (빨간색, 반투명)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
        
        marker_array.markers.append(marker)
        self.obstacle_marker_pub.publish(marker_array)
        
        # 로그에 장애물 정보 추가
        self.get_logger().info(
            f"Obstacle UTM: x={self.obstacle_x:.2f}, y={self.obstacle_y:.2f}, "
            f"a={self.obstacle_a:.2f}m, b={self.obstacle_b:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    simulation_pub = SimulationPub()
    rclpy.spin(simulation_pub)
    simulation_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 