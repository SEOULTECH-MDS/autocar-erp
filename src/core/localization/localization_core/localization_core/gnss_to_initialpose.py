#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from math import atan2, hypot
from autocar_utils.euler_from_quaternion import euler_from_quaternion
from autocar_utils.yaw_to_quaternion import yaw_to_quaternion


class GnssToInitialPose(Node):
    def __init__(self):
        super().__init__('gnss_to_initialpose')
        self.declare_parameter('gnss_pose_topic', '/sensing/gnss/pose_with_covariance')
        self.declare_parameter('initialpose_topic', '/initialpose3d')
        self.declare_parameter('once', True)
        self.declare_parameter('fix_velocity_topic', '/ublox_gps_node/fix_velocity')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('speed_threshold', 0.5)

        gnss_topic = self.get_parameter('gnss_pose_topic').get_parameter_value().string_value
        self.initialpose_topic = self.get_parameter('initialpose_topic').get_parameter_value().string_value
        self.once = bool(self.get_parameter('once').value)
        self.fix_vel_topic = self.get_parameter('fix_velocity_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.speed_thresh = float(self.get_parameter('speed_threshold').value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 1)
        self.create_subscription(PoseWithCovarianceStamped, gnss_topic, self.on_gnss, 10)
        self.sent = False

        # caches for yaw estimation
        self.last_fix_vel: TwistWithCovarianceStamped | None = None
        self.last_imu: Imu | None = None
        self.create_subscription(TwistWithCovarianceStamped, self.fix_vel_topic, self.on_fixvel, 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)

    def on_gnss(self, msg: PoseWithCovarianceStamped) -> None:
        if self.once and self.sent:
            return
        # estimate yaw from fix_velocity when available; otherwise fall back to IMU yaw
        yaw = None
        if self.last_fix_vel is not None:
            vx = float(self.last_fix_vel.twist.twist.linear.x)
            vy = float(self.last_fix_vel.twist.twist.linear.y)
            if hypot(vx, vy) > self.speed_thresh:
                yaw = atan2(vy, vx)

        if yaw is None and self.last_imu is not None:
            yaw = euler_from_quaternion(
                self.last_imu.orientation.x,
                self.last_imu.orientation.y,
                self.last_imu.orientation.z,
                self.last_imu.orientation.w,
            )

        out = msg
        if yaw is not None:
            out.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.pub.publish(out)
        self.sent = True
        if self.once:
            self.get_logger().info('Published initial pose from GNSS; shutting down')
            # small delay then shutdown
            rclpy.shutdown()

    def on_fixvel(self, msg: TwistWithCovarianceStamped) -> None:
        self.last_fix_vel = msg

    def on_imu(self, msg: Imu) -> None:
        self.last_imu = msg


def main(args=None):
    rclpy.init(args=args)
    node = GnssToInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


