#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node 

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, PointCloud2
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, QuaternionStamped, PoseArray,TwistWithCovarianceStamped
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
        self.tracker_sub = self.create_subscription(Twist, '/autocar/cmd_vel',self.cmd_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/carla/hero/gnss',self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/carla/hero/imu',self.imu_callback, 10)
        self.speed_sub = self.create_subscription(Float32, '/carla/hero/speedometor',self.speed_callback, 10)

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
        self.cmd_velocity = cmd_msg.linear.x
        self.cmd_steering = cmd_msg.angular.z
        
        carla_cmd = CarlaEgoVehicleControl()
        carla_cmd.throttle = self.cmd_velocity
        carla_cmd.steer = self.cmd_steering
        self.tracker_pub.publish(carla_cmd)
    
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