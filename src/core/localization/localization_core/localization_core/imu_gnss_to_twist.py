#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
import pyproj
import math


class ImuGnssToTwist(Node):
    def __init__(self):
        super().__init__('imu_gnss_to_twist')

        # Parameters
        self.declare_parameter('input_fix_topic', '/ublox_gps_node/fix')
        self.declare_parameter('input_imu_topic', '/imu/data')
        self.declare_parameter('map_origin_lat', 37.239205)
        self.declare_parameter('map_origin_lon', 126.773193)
        self.declare_parameter('output_topic', '/localization/yabloc/twist_with_covariance')
        self.declare_parameter('yaw_rate_sign', 1.0)
        self.declare_parameter('lowpass_alpha', 0.5)  # 0..1, higher = more smoothing
        self.declare_parameter('max_dt_sec', 0.5)
        self.declare_parameter('min_speed_for_heading_mps', 0.5)
        self.declare_parameter('restamp_to_clock', False)

        # IO topics
        fix_topic = self.get_parameter('input_fix_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('input_imu_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # UTM projector based on map origin
        lat0 = float(self.get_parameter('map_origin_lat').value)
        lon0 = float(self.get_parameter('map_origin_lon').value)
        zone = int((lon0 + 180) / 6) + 1
        north = 1 if lat0 >= 0 else 0
        proj_string = f'+proj=utm +zone={zone} {"+north" if north else "+south"} +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        self.utm = pyproj.Proj(proj_string)

        # State
        self.prev_xy = None  # (x, y, stamp_sec)
        self.filtered_speed = 0.0
        self.last_yaw = 0.0
        self.last_yawrate = 0.0
        self.yaw_rate_sign = float(self.get_parameter('yaw_rate_sign').value)
        self.alpha = float(self.get_parameter('lowpass_alpha').value)
        self.max_dt = float(self.get_parameter('max_dt_sec').value)
        self.min_speed_for_heading = float(self.get_parameter('min_speed_for_heading_mps').value)

        # IO
        self.pub = self.create_publisher(TwistWithCovarianceStamped, output_topic, 10)
        self.create_subscription(NavSatFix, fix_topic, self.on_fix, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, 50)

    def on_imu(self, msg: Imu) -> None:
        # yaw from quaternion (ENU)
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.last_yaw = math.atan2(siny_cosp, cosy_cosp)

        # yaw rate (z), allow sign flip
        self.last_yawrate = float(msg.angular_velocity.z) * self.yaw_rate_sign

    def on_fix(self, msg: NavSatFix) -> None:
        try:
            # Convert to map-ENU meters
            x_m, y_m = self.utm(msg.longitude, msg.latitude)
        except Exception:
            return

        stamp_sec = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        # Speed from displacement
        speed_mps = self.filtered_speed
        if self.prev_xy is not None:
            prev_x, prev_y, prev_t = self.prev_xy
            dt = max(1e-3, min(self.max_dt, stamp_sec - prev_t))
            if dt > 0.0:
                dx = float(x_m - prev_x)
                dy = float(y_m - prev_y)
                inst_speed = math.hypot(dx, dy) / dt
                # Low-pass filter
                a = max(0.0, min(1.0, self.alpha))
                speed_mps = a * self.filtered_speed + (1.0 - a) * inst_speed

        self.filtered_speed = speed_mps
        self.prev_xy = (x_m, y_m, stamp_sec)

        # Project map-frame velocity to body x using yaw if moving sufficiently
        vx_body = speed_mps
        if self.prev_xy is not None and speed_mps >= self.min_speed_for_heading:
            # approximate heading = IMU yaw; keep body x aligned to yaw
            vx_body = speed_mps  # forward along body x

        # Publish twist
        out = TwistWithCovarianceStamped()
        # Use original bag timestamp by default; optionally restamp to /clock
        try:
            if bool(self.get_parameter('restamp_to_clock').value):
                out.header.stamp = self.get_clock().now().to_msg()
            else:
                out.header.stamp = msg.header.stamp
        except Exception:
            out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'
        out.twist.twist.linear.x = float(vx_body)
        out.twist.twist.linear.y = 0.0
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = float(self.last_yawrate)

        # Covariance placeholders; predictor overrides with static params
        out.twist.covariance[0] = 0.25  # var(vx)
        out.twist.covariance[35] = (2.0 * math.pi / 180.0) ** 2  # var(yawrate)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGnssToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


