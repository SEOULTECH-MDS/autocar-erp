#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped


def t2f(t: Time) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


class FixVelImuToTwist(Node):
    def __init__(self):
        super().__init__('fixvel_imu_to_twist')

        # Parameters
        self.declare_parameter('fix_velocity_topic', '/ublox_gps_node/fix_velocity')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('output_topic', '/localization/twist_estimator/twist_with_covariance')
        # Backward compat: sync_timeout_sec kept as alias
        self.declare_parameter('sync_timeout_sec', 0.2)
        self.declare_parameter('max_skew_sec', 0.5)
        self.declare_parameter('allow_fix_only', True)
        self.declare_parameter('use_speed_magnitude', True)
        self.declare_parameter('yaw_rate_sign', 1.0)

        fix_topic = self.get_parameter('fix_velocity_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        # prefer max_skew_sec if provided, fall back to sync_timeout_sec
        max_skew = self.get_parameter('max_skew_sec').value
        if isinstance(max_skew, float):
            self.max_skew = float(max_skew)
        else:
            self.max_skew = float(self.get_parameter('sync_timeout_sec').value)
        self.allow_fix_only = bool(self.get_parameter('allow_fix_only').value)
        self.use_speed_mag = bool(self.get_parameter('use_speed_magnitude').value)
        try:
            self.yaw_sign = float(self.get_parameter('yaw_rate_sign').value)
        except Exception:
            self.yaw_sign = 1.0

        # State (latest samples)
        self.last_fix: TwistWithCovarianceStamped | None = None
        self.last_imu: Imu | None = None

        # IO
        self.pub = self.create_publisher(TwistWithCovarianceStamped, out_topic, 10)
        self.create_subscription(TwistWithCovarianceStamped, fix_topic, self.on_fixvel, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, 10)

        # periodic fuse in case one stream is faster
        self.timer = self.create_timer(0.02, self.try_publish)

    def on_fixvel(self, msg: TwistWithCovarianceStamped) -> None:
        self.last_fix = msg
        self.try_publish()

    def on_imu(self, msg: Imu) -> None:
        self.last_imu = msg
        self.try_publish()

    def try_publish(self) -> None:
        if self.last_fix is None or self.last_imu is None:
            return

        fix_t = t2f(self.last_fix.header.stamp)
        imu_t = t2f(self.last_imu.header.stamp)
        skew = abs(fix_t - imu_t)

        out = TwistWithCovarianceStamped()
        out.header.frame_id = 'base_link'

        # Choose stamp sensibly
        out.header.stamp = self.last_fix.header.stamp if fix_t >= imu_t else self.last_imu.header.stamp

        # Linear speed from GNSS fix velocity
        vx = 0.0
        try:
            if self.use_speed_mag:
                vx_x = float(self.last_fix.twist.twist.linear.x)
                vx_y = float(self.last_fix.twist.twist.linear.y)
                vx = (vx_x * vx_x + vx_y * vx_y) ** 0.5
            else:
                vx = float(self.last_fix.twist.twist.linear.x)
        except Exception:
            vx = 0.0

        # Yaw rate from IMU (Z)
        wz = 0.0
        try:
            wz = float(self.last_imu.angular_velocity.z) * self.yaw_sign
        except Exception:
            wz = 0.0

        if skew > self.max_skew and not self.allow_fix_only:
            # too far apart, and no fallback permitted
            return

        # If skew is large, still publish with available components (vx from fix, wz possibly 0)
        out.twist.twist.linear.x = vx
        out.twist.twist.angular.z = wz if skew <= self.max_skew else 0.0

        # Covariances: start with zeros
        out.twist.covariance = [0.0] * 36
        # Use GNSS velocity covariance if provided
        try:
            vx_cov = float(self.last_fix.twist.covariance[0])
            if vx_cov <= 0.0:
                vx_cov = 0.1
        except Exception:
            vx_cov = 0.1
        out.twist.covariance[0] = vx_cov

        # IMU yaw rate covariance if available
        try:
            wz_cov = float(self.last_imu.angular_velocity_covariance[8])
            if wz_cov <= 0.0:
                wz_cov = 0.05
        except Exception:
            wz_cov = 0.05
        out.twist.covariance[35] = wz_cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FixVelImuToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


