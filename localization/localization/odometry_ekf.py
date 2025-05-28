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
from autocar_nav.normalise_angle import normalise_angle
from geometry_msgs.msg import PolygonStamped, Point32

from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class ExtendedKalmanFilter:
    def __init__(self):
        self.state = np.zeros(5)  # [x, y, yaw, v, yaw_rate]
        self.P = np.eye(5) * 1000  # 초기 불확실성
        self.Q = np.eye(5) * 0.1  # 프로세스 노이즈
        self.R_gps = np.eye(2) * 0.1  # GPS 측정 노이즈
        self.R_imu = np.eye(2) * 0.01  # IMU 측정 노이즈
        self.R_speed = np.array([[0.1]])  # 속도 노이즈

    def predict(self, dt):
        # 상태 예측
        self.state[0] += self.state[3] * np.cos(self.state[2]) * dt
        self.state[1] += self.state[3] * np.sin(self.state[2]) * dt
        self.state[2] += self.state[4] * dt

        # 자코비안 계산
        F = np.eye(5)
        F[0, 2] = -self.state[3] * np.sin(self.state[2]) * dt
        F[0, 3] = np.cos(self.state[2]) * dt
        F[1, 2] = self.state[3] * np.cos(self.state[2]) * dt
        F[1, 3] = np.sin(self.state[2]) * dt
        F[2, 4] = dt

        # 공분산 예측
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, measurement):
        H = np.zeros((2, 5))
        H[0, 0] = H[1, 1] = 1

        y = measurement - self.state[:2]
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_imu(self, yaw, yaw_rate):
        H = np.zeros((2, 5))
        H[0, 2] = H[1, 4] = 1

        y = np.array([yaw, yaw_rate]) - self.state[[2, 4]]
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
    
    def update_speed(self, measured_speed):
        try:
            H = np.zeros((1, 5))
            H[0, 3] = 1  # 속도 상태(v)에 대한 관측 행렬

            y = np.array([measured_speed - self.state[3]]).reshape(-1, 1)
            S = H @ self.P @ H.T + self.R_speed
            K = (self.P @ H.T @ np.linalg.inv(S)).reshape(-1, 1)

            # 상태 업데이트
            self.state += (K @ y).flatten()

            # 공분산 업데이트
            self.P = (np.eye(5) - K @ H) @ self.P

        except ValueError as e:
            print(f"ValueError in update_speed: {e}")
            print(f"K shape: {K.shape if 'K' in locals() else 'undefined'}")
            print(f"y shape: {y.shape if 'y' in locals() else 'undefined'}")
            print(f"self.state shape: {self.state.shape}")
            print(f"H shape: {H.shape}")
            print(f"S shape: {S.shape if 'S' in locals() else 'undefined'}")


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.ekf = ExtendedKalmanFilter()
        self.last_time = self.get_clock().now()
        self.gps_pose = Odometry()
        self.gps_pose.header.frame_id = 'world'
        self.global_yaw = 0.0
        self.encoder_speed = 0.0
        self.speed = 0.0
        self.yaw_offset = 0.0
        self.vehicle_length = 2.0
        self.vehicle_width = 1.0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 서브스크라이버 초기화
        self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.callback_gps, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.callback_imu, 10)
        self.speed_sub = self.create_subscription(TwistWithCovarianceStamped, "/ublox_gps_node/fix_velocity", self.callback_speed, 10)
        self.encoder_speed_sub = self.create_subscription(Float64, "/encoder_speed", self.callback_encoder_speed, 10)
        #초기 yaw 설정
        self.init_orientation_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback_init_orientation, 10)

        # 퍼블리셔 초기화
        self.odom_pub = self.create_publisher(Odometry, '/autocar/location', 10)
        
        # self.location_viz_pub = self.create_publisher(PolygonStamped, '/autocar/viz_location', 10)

        # 타이머 초기화
        self.timer = self.create_timer(0.05, self.publish_odometry)

    def callback_gps(self, gps_msg):
        # GPS 좌표를 UTM 좌표로 변환
        x, y = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)
        self.get_logger().info(
            f"\nx(gps raw): {x},\n"
            f"y(gps raw): {y},\n"
        )
        self.ekf.update_gps(np.array([x, y]))
        
    def callback_imu(self, imu_msg):
        # IMU 데이터를 이용하여 자동차의 yaw 각도 계산
        local_yaw = euler_from_quaternion(imu_msg.orientation.x, imu_msg.orientation.y,
                                    imu_msg.orientation.z, imu_msg.orientation.w)
        yaw_rate = imu_msg.angular_velocity.z

        global_yaw = normalise_angle(local_yaw + self.yaw_offset)
        self.ekf.update_imu(global_yaw, yaw_rate)

    def callback_encoder_speed(self, encoder_msg):
        
        self.encoder_speed = encoder_msg.data

    def callback_speed(self, speed_msg):
        
        measured_speed = speed_msg.twist.twist.linear.x
        self.ekf.update_speed(measured_speed)
    
    def callback_init_orientation(self, init_pose_msg):
        transform = self.tf_buffer.lookup_transform('world', init_pose_msg.header.frame_id, rclpy.time.Time())
        # 좌표 변환
        global_init_pose_msg = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(init_pose_msg, transform)
        global_yaw = euler_from_quaternion(global_init_pose_msg.pose.pose.orientation.x, global_init_pose_msg.pose.pose.orientation.y, \
                                           global_init_pose_msg.pose.pose.orientation.z, global_init_pose_msg.pose.pose.orientation.w)
       
        local_yaw = euler_from_quaternion(self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y,\
                                          self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w)

        self.yaw_offset += global_yaw - local_yaw

    def publish_odometry(self):
        # GPS 좌표와 IMU 데이터를 이용하여 오도메트리 데이터 생성
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.ekf.predict(dt)

        # EKF 상태를 Odometry 메시지로 변환
        self.gps_pose.pose.pose.position.x = self.ekf.state[0]
        self.gps_pose.pose.pose.position.y = self.ekf.state[1]
        self.gps_pose.pose.pose.orientation = yaw_to_quaternion(self.ekf.state[2])
        self.gps_pose.twist.twist.linear.x = self.ekf.state[3]
        self.gps_pose.twist.twist.angular.z = self.ekf.state[4]

        # 공분산 행렬 업데이트
        self.gps_pose.pose.covariance = np.pad(self.ekf.P[:3, :3], (0, 3)).flatten()
        self.gps_pose.twist.covariance = np.pad(self.ekf.P[3:, 3:], (0, 4)).flatten()

        self.odom_pub.publish(self.gps_pose)

        self.get_logger().info(
            f"\nx: {self.gps_pose.pose.pose.position.x},\n"
            f"y: {self.gps_pose.pose.pose.position.y},\n"
            f"yaw: {self.ekf.state[2] / np.pi * 180.0} degree,\n"
            f"speed: {self.ekf.state[3]},\n"
            f"yaw_rate: {self.ekf.state[4]}\n"
        )

def latlon_to_utm(lat, lon):
    # WGS84 좌표계를 UTM 좌표계로 변환
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
