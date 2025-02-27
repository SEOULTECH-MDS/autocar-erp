#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyproj
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped, PoseArray,TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.gps_pose = Odometry()
        self.gps_pose.header.frame_id = 'world'
        self.global_yaw = 0.0
        self.speed = 0.0

        # 서브스크라이버 초기화
        self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps/fix', self.callback_gps, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.callback_imu, 10)
        #self.speed_sub = self.create_subscription(TwistWithCovarianceStamped, "/ublox_gps/fix_velocity", self.callback_speed, 10)
        #self.speed_sub = self.create_subscription(Float64, "/cur_speed", self.callback_speed, 10)

        # 퍼블리셔 초기화
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 타이머 초기화
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def callback_gps(self, gps_msg):
        # GPS 좌표를 UTM 좌표로 변환
        self.gps_pose.pose.pose.position.x, self.gps_pose.pose.pose.position.y = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)

    def callback_imu(self, imu_msg):
        # IMU 데이터를 이용하여 자동차의 yaw 각도 계산
        local_yaw = euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])[2]
        self.global_yaw = normalize_angle(local_yaw)
        self.gps_pose.pose.pose.orientation = yaw_to_quaternion(self.global_yaw)

    def callback_speed(self, speed_msg):
        self.speed = np.sqrt(speed_msg.twist.twist.linear.x **2 + speed_msg.twist.twist.linear.y**2)
        # self.speed = speed_msg.data
        # 테스트 해보고 윗줄 삭제

    def publish_odometry(self):
        # GPS 좌표와 IMU 데이터를 이용하여 오도메트리 데이터 생성
        #self.gps_pose.twist.twist.linear.x = self.speed
        self.odom_pub.publish(self.gps_pose)

def latlon_to_utm(lat, lon):
    # WGS84 좌표계를 UTM 좌표계로 변환
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()