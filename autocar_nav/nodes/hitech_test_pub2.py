#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import pyproj
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
import numpy as np
import math

# 차량 파라미터
DT = 0.2  # 주기 
WB = 1.566  # 차량 휠베이스 (m)

def latlon_to_utm(lat, lon):
    # WGS84 좌표계를 UTM 좌표계로 변환
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

class HitechTestPub(Node):
    def __init__(self):
        super().__init__('hitech_test_pub')
        self.sub_cmd = self.create_subscription(AckermannDriveStamped, '/erp/cmd_vel', self.cmd_callback, 10)
        
        self.publisher_odometry = self.create_publisher(Odometry, '/autocar/location', 10)  # Odometry 퍼블리셔 추가
        self.get_logger().info("hitech_test_pub START")

        self.latitude = 37.632010 # 초기 위도
        self.longitude = 127.076008  # 초기 경도

        self.x, self.y = latlon_to_utm(self.latitude, self.longitude)  # 초기 x 좌표, y 좌표
        self.yaw = np.deg2rad(90.0) # 초기 yaw 값 (라디안 단위)
        self.velocity = 0.0
        self.steering_angle = 0.0  # 조향각
        
        self.create_timer(DT, self.update_and_publish)  # 10Hz 주기로 데이터 업데이트 및 퍼블리시
    
    def cmd_callback(self, msg):
        self.velocity = msg.drive.speed
        self.steering_angle = msg.drive.steering_angle

    def update_and_publish(self):
        self.update_state()
        self.publish_odometry()  # Odometry 퍼블리시
    
    def update_state(self):
        # 차량의 위치 및 방향 업데이트
        self.x += self.velocity * math.cos(self.yaw) * DT
        self.y += self.velocity * math.sin(self.yaw) * DT
        
        # Yaw 값 업데이트 (조향각 반영)
        if abs(self.steering_angle) > 1e-3:  # 조향각이 0이 아닐 경우 회전 반영
            self.yaw = self.yaw + self.velocity / WB * math.tan(self.steering_angle) * DT
        
        self.yaw = self.yaw % (2 * np.pi)  # 0 ~ 2pi 범위 유지
    
    def publish_odometry(self):
        # Odometry 메시지 생성
        msg = Odometry()
        msg.header.frame_id = 'world'
        
        # 위치와 방향 설정
        msg.pose.pose.position.x = self.x  # x 좌표
        msg.pose.pose.position.y = self.y  # y 좌표
        q = yaw_to_quaternion(self.yaw)
        msg.pose.pose.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        
        # 속도와 회전 속도 설정
        msg.twist.twist.linear.x = self.velocity * math.cos(self.yaw)
        msg.twist.twist.linear.y = self.velocity * math.sin(self.yaw)
        msg.twist.twist.angular.z = self.velocity / (WB * math.tan(self.steering_angle)) if abs(self.steering_angle) > 1e-3 else 0.0
        
        # Odometry 메시지 퍼블리시
        self.publisher_odometry.publish(msg)
        self.get_logger().info(f"Odometry published: x={msg.pose.pose.position.x:.6f}, y={msg.pose.pose.position.y:.6f}, yaw={np.rad2deg(self.yaw):.2f} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = HitechTestPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
