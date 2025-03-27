#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import numpy as np
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped


# gps , imu 데이터 테스트용 노드
class HitechTestPub(Node):
    def __init__(self):
        super().__init__('hitech_test_pub')
        self.publisher_gps = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)


        self.get_logger().info("hitech_test_pub START")

        self.yaw = 1.5  # 초기 yaw 값 (라디안 단위)
        self.yaw_rate = np.deg2rad(5.0)  # 1초에 1도 회전 (라디안 변환)
        
        self.latitude = 37.632010
        self.longitude = 127.076008  # 하이테크 텐트 앞

        # self.latitude = 37.63283317
        # self.longitude = 127.07819321 # 붕어방 도로
        self.velocity = 0.000001

        self.create_timer(0.1, self.publish_gps)  # GPS 데이터 퍼블리시
        self.create_timer(0.1, self.publish_imu)  # IMU 데이터 퍼블리시


    # GPS 메시지 퍼블리싱
    def publish_gps(self):
        msg = NavSatFix()

        # x, y 위도, 경도 값 증가
        # self.latitude += self.velocity * 2.0
        # self.longitude += self.velocity * 0.8  
        
        msg.latitude = self.latitude
        msg.longitude = self.longitude

        self.publisher_gps.publish(msg)
        self.get_logger().info(f"GPS published: lat={msg.latitude}, lon={msg.longitude}")

    # IMU 메시지 퍼블리싱
    def publish_imu(self):
        msg = Imu()
        
        # 시계 반대 방향으로 yaw 증가
        # self.yaw += self.yaw_rate * (0.1)  # 20Hz 주기로 업데이트
        # self.yaw = self.yaw % (2 * np.pi)  # 0 ~ 2pi 범위 유지

        # Quaternion 변환
        q = yaw_to_quaternion(self.yaw)
        msg.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

        # 각속도 설정 (yaw 축만 회전)
        # msg.angular_velocity.z = self.yaw_rate

        self.publisher_imu.publish(msg)
        self.get_logger().info(f"IMU published: yaw={np.rad2deg(self.yaw):.2f} degrees")


def main(args=None):
    rclpy.init(args=args)
    hitech_test_pub = HitechTestPub()
    rclpy.spin(hitech_test_pub)
    hitech_test_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()