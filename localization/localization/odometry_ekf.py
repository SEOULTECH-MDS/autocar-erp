#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyproj
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped, PoseArray,TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from autocar_nav.normalise_angle import normalise_angle
from geometry_msgs.msg import PolygonStamped, Point32

from localization.ekf import EKFNavINS, GPSCoordinate, GPSVelocity, IMUData

from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.ekf = EKFNavINS()
        self.imu_angular_velocity_x = 0.0
        self.imu_angular_velocity_y = 0.0
        self.imu_angular_velocity_z = 0.0
        self.global_yaw = 0.0
        self.encoder_speed = 0.0
        self.gps_speed = 0.0
        self.ekf_speed = 0.0
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
        self.speed_set_pub = self.create_publisher(Float64MultiArray, '/speed_set', 10)
        # self.location_viz_pub = self.create_publisher(PolygonStamped, '/autocar/viz_location', 10)

        # 타이머 초기화
        self.timer1 = self.create_timer(0.1, self.publish_odometry)
        self.timer2 = self.create_timer(0.1, self.publish_speed_set)

    def callback_gps(self, gps_msg):
        gps_coor = GPSCoordinate(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)
        self.ekf.gps_coordinate_update_ekf(gps_coor)
        
    def callback_imu(self, imu_msg):
        imu_data = IMUData(
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
            0, 0, 0  # 자기장 데이터가 없는 경우 0으로 설정
        )
        self.imu_angular_velocity_x = imu_msg.angular_velocity.x
        self.imu_angular_velocity_y = imu_msg.angular_velocity.y
        self.imu_angular_velocity_z = imu_msg.angular_velocity.z
        self.ekf.imu_update_ekf(self.get_clock().now().nanoseconds / 1e6, imu_data)

    def callback_encoder_speed(self, encoder_msg):
        self.encoder_speed = encoder_msg.data

    def callback_speed(self, speed_msg):
        self.gps_speed = np.sqrt(speed_msg.twist.twist.linear.x **2 + speed_msg.twist.twist.linear.y**2)

    def callback_init_orientation(self, init_pose_msg):
        transform = self.tf_buffer.lookup_transform('world', init_pose_msg.header.frame_id, rclpy.time.Time())
        # 좌표 변환
        global_init_pose_msg = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(init_pose_msg, transform)
        global_yaw = euler_from_quaternion(global_init_pose_msg.pose.pose.orientation.x, global_init_pose_msg.pose.pose.orientation.y, \
                                           global_init_pose_msg.pose.pose.orientation.z, global_init_pose_msg.pose.pose.orientation.w)
       
        local_yaw = euler_from_quaternion(self.ekf.quat[1, 0], self.ekf.quat[2, 0], \
                                          self.ekf.quat[3, 0], self.ekf.quat[0, 0])

        self.yaw_offset += global_yaw - local_yaw

    # EKF 보정된 위치, 속도를 odometry 메시지로 publish
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = latlon_to_utm(self.ekf.lat_ins, self.ekf.lon_ins)
        odom_msg.pose.pose.position.z = self.ekf.alt_ins

        local_yaw = euler_from_quaternion(self.ekf.quat[1, 0], self.ekf.quat[2, 0], \
                                          self.ekf.quat[3, 0], self.ekf.quat[0, 0])
        global_yaw = local_yaw + self.yaw_offset
        self.global_yaw = normalise_angle(global_yaw)
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.global_yaw)
        
        odom_msg.twist.twist.angular.x = self.imu_angular_velocity_x
        odom_msg.twist.twist.angular.y = self.imu_angular_velocity_y
        odom_msg.twist.twist.angular.z = self.imu_angular_velocity_z

        odom_msg.twist.twist.linear.x = self.ekf.vn_ins
        odom_msg.twist.twist.linear.y = self.ekf.ve_ins
        odom_msg.twist.twist.linear.z = self.ekf.vd_ins

        self.ekf_speed = np.sqrt(self.ekf.vn_ins ** 2 + self.ekf.ve_ins ** 2)
        
        self.odom_pub.publish(odom_msg)
        
        self.get_logger().info(
            f"\nx: {odom_msg.pose.pose.position.x},\n"
            f"y: {odom_msg.pose.pose.position.y},\n"
            f"yaw: {self.global_yaw / np.pi * 180.0} degree"
        )

    # GPS, Encoder, EKF 속도 비교를 위해 publish
    def publish_speed_set(self):
        speed_set = Float64MultiArray()
        speed_set.data = [self.gps_speed, self.encoder_speed, self.ekf_speed]  # GPS, Encoder, EKF 속도
        self.speed_set_pub.publish(speed_set)
        self.get_logger().info(f'Published: {speed_set.data}')

        # # rviz 차량 위치 시각화
        # corners = self.get_vehicle_corners(self.global_yaw)

        # # Polygon 메시지 생성 및 퍼블리시
        # polygon_msg = PolygonStamped()
        # polygon_msg.header.stamp = self.get_clock().now().to_msg()
        # polygon_msg.header.frame_id = "base_link"   # 차량의 base_link 좌표 중심으로 시각화
        # for corner in corners:
        #     point = Point32()
        #     point.x, point.y = corner
        #     polygon_msg.polygon.points.append(point)

        # self.location_viz_pub.publish(polygon_msg)

    # def get_vehicle_corners(self, yaw):
    #     """ 차량 중심과 yaw를 기준으로 사각형 네 꼭짓점 좌표 계산 """
    #     half_length = self.vehicle_length / 2
    #     half_width = self.vehicle_width / 2

    #     # 차량 좌표 기준 네 개 꼭짓점 (로컬 좌표)
    #     corners_local = np.array([
    #         [ half_length,  half_width],  # front-right
    #         [ half_length, -half_width],  # front-left
    #         [-half_length, -half_width],  # rear-left
    #         [-half_length,  half_width]   # rear-right
    #     ])

    #     # 회전 변환 (yaw 적용)
    #     rotation_matrix = np.array([
    #         [np.cos(yaw), -np.sin(yaw)],
    #         [np.sin(yaw),  np.cos(yaw)]
    #     ])
    #     rotated_corners = (rotation_matrix @ corners_local.T).T

    #     return rotated_corners

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
