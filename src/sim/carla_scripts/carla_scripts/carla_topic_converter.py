#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node 

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, PointCloud2
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, QuaternionStamped, PoseArray, TwistWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32, Int16, Int32, Float64MultiArray, String, Bool
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32

from carla_msgs.msg import CarlaEgoVehicleControl

class CarlaTopicConverter(Node):
    def __init__(self):
        super().__init__('carla_topic_converter')

        # Subscribe to the topics
        self.tracker_sub = self.create_subscription(Twist, '/autocar/cmd_vel', self.cmd_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/carla/hero/gnss', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/carla/hero/imu', self.imu_callback, 10)
        self.speed_sub = self.create_subscription(Float32, '/carla/hero/speedometer', self.speed_callback, 10)

        # Publish to the topics
        self.tracker_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.speed_pub = self.create_publisher(TwistWithCovarianceStamped, "/ublox_gps_node/fix_velocity", 10)

        # Variables
        self.cmd_velocity = None
        self.cmd_steering = None
        self.gps_latitude = None
        self.gps_longitude = None
        self.gps_altitude = None
        self.imu_orientation = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = None
        self.speed = None
        
    def cmd_callback(self, cmd_msg):
        # Twist 메시지를 CARLA 제어 명령으로 변환
        carla_cmd = CarlaEgoVehicleControl()
        
        # 속도 변환 (m/s -> km/h)
        velocity = cmd_msg.linear.x * 3.6  # m/s to km/h
        
        # 조향각 변환 (rad -> deg)
        steering = cmd_msg.angular.z * 180.0 / 3.14159  # rad to deg
        
        # CARLA 제어 명령 설정
        carla_cmd.throttle = max(0.0, min(1.0, velocity / 20.0))  # 속도를 0-1 범위로 정규화
        carla_cmd.steer = max(-1.0, min(1.0, steering / 70.0))    # 조향각을 -1~1 범위로 정규화
        carla_cmd.brake = 0.0  # 기본값
        carla_cmd.hand_brake = False
        carla_cmd.reverse = False
        carla_cmd.manual_gear_shift = False
        carla_cmd.gear = 1  # 전진 기어
        
        # 제어 명령 발행
        self.tracker_pub.publish(carla_cmd)
        
        # 디버그 로그
        self.get_logger().info(f'CARLA 제어 명령: 속도={velocity:.2f}km/h, 조향각={steering:.2f}deg')
    
    def gps_callback(self, gps_msg):
        self.gps_latitude = gps_msg.latitude
        self.gps_longitude = gps_msg.longitude
        self.gps_altitude = gps_msg.altitude

        carla_gps = NavSatFix()
        carla_gps.latitude = self.gps_latitude
        carla_gps.longitude = self.gps_longitude
        carla_gps.altitude = self.gps_altitude
        self.gps_pub.publish(carla_gps)
    
    def imu_callback(self, imu_msg):
        self.imu_orientation = imu_msg.orientation
        self.imu_angular_velocity = imu_msg.angular_velocity
        self.imu_linear_acceleration = imu_msg.linear_acceleration

        carla_imu = Imu()
        carla_imu.orientation = self.imu_orientation
        carla_imu.angular_velocity = self.imu_angular_velocity
        carla_imu.linear_acceleration = self.imu_linear_acceleration
        self.imu_pub.publish(carla_imu)

    def speed_callback(self, speed_msg):
        self.speed = speed_msg.data
        carla_speed = TwistWithCovarianceStamped()
        carla_speed.twist.twist.linear.x = self.speed
        self.speed_pub.publish(carla_speed)
    
def main(args=None):
    rclpy.init(args=args)
    try:
        carla_topic_converter = CarlaTopicConverter()
        rclpy.spin(carla_topic_converter)
    finally:
        carla_topic_converter.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()