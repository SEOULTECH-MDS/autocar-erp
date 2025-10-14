#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import pyproj
import math


class SingleGnssImuPose(Node):
    def __init__(self):
        super().__init__('single_gnss_imu_pose')

        # Parameters
        self.declare_parameter('input_fix_topic', '/ublox_gps_node/fix')
        self.declare_parameter('input_imu_topic', '/imu/data')
        self.declare_parameter('map_origin_lat', 37.239205)
        self.declare_parameter('map_origin_lon', 126.773193)
        self.declare_parameter('output_pose_topic', '/sensing/gnss/pose_with_covariance')
        self.declare_parameter('yaw_bias_deg', 0.0)
        self.declare_parameter('auto_heading_align', True)
        self.declare_parameter('min_align_displacement_m', 1.5)
        self.declare_parameter('max_align_bias_deg', 20.0)
        self.declare_parameter('align_once', True)

        fix_topic = self.get_parameter('input_fix_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('input_imu_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_pose_topic').get_parameter_value().string_value

        # UTM projector (zone is derived from origin)
        lat0 = float(self.get_parameter('map_origin_lat').value)
        lon0 = float(self.get_parameter('map_origin_lon').value)
        zone = int((lon0 + 180) / 6) + 1
        north = 1 if lat0 >= 0 else 0
        proj_string = f'+proj=utm +zone={zone} {"+north" if north else "+south"} +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        self.utm = pyproj.Proj(proj_string)
        self.map_origin_x, self.map_origin_y = self.utm(lon0, lat0)

        # State
        self.last_yaw = 0.0
        # bias in radians
        try:
            self.yaw_bias = float(self.get_parameter('yaw_bias_deg').value) * math.pi / 180.0
        except Exception:
            self.yaw_bias = 0.0

        # Auto heading alignment settings
        self.auto_align = bool(self.get_parameter('auto_heading_align').value)
        self.min_align_disp = float(self.get_parameter('min_align_displacement_m').value)
        self.max_align_bias = float(self.get_parameter('max_align_bias_deg').value) * math.pi / 180.0
        self.align_once = bool(self.get_parameter('align_once').value)
        self.aligned = False
        self.prev_fix_xy = None  # type: tuple[float, float] | None

        # IO
        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        self.create_subscription(NavSatFix, fix_topic, self.on_fix, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, 10)

    def on_imu(self, msg: Imu) -> None:
        # derive yaw from quaternion (assuming ENU)
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        # yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.last_yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_fix(self, msg: NavSatFix) -> None:
        # basic quality gate: covariance finite (if provided)
        try:
            if len(msg.position_covariance) > 0 and any(math.isnan(float(c)) for c in msg.position_covariance):
                return
        except Exception:
            pass

        x, y = self.utm(msg.longitude, msg.latitude)
        px = float(x - self.map_origin_x)
        py = float(y - self.map_origin_y)

        # Auto heading alignment from displacement course
        if self.auto_align and (not self.aligned or not self.align_once):
            if self.prev_fix_xy is not None:
                dx = px - self.prev_fix_xy[0]
                dy = py - self.prev_fix_xy[1]
                disp = math.hypot(dx, dy)
                if disp >= self.min_align_disp:
                    course = math.atan2(dy, dx)  # map frame ENU: x-east, y-north
                    # normalize difference to [-pi, pi]
                    diff = course - self.last_yaw
                    while diff > math.pi:
                        diff -= 2.0 * math.pi
                    while diff < -math.pi:
                        diff += 2.0 * math.pi
                    # clamp to reasonable bound
                    if abs(diff) > self.max_align_bias:
                        diff = self.max_align_bias if diff > 0 else -self.max_align_bias
                    self.yaw_bias = diff
                    self.aligned = True
                    self.get_logger().info(f"Auto heading aligned: course={course:.3f}, imu_yaw={self.last_yaw:.3f}, bias={self.yaw_bias:.3f} rad")
            self.prev_fix_xy = (px, py)

        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = 'map'
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        # Ignore GPS altitude; keep z at ground level
        out.pose.pose.position.z = 0.0

        # yaw only (roll/pitch ignored)
        yaw_biased = self.last_yaw + self.yaw_bias
        cy = math.cos(yaw_biased * 0.5)
        sy = math.sin(yaw_biased * 0.5)
        out.pose.pose.orientation.w = cy
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = sy

        # covariance: use NavSatFix covariance[0] if available; otherwise fallback
        try:
            cov_xy = float(msg.position_covariance[0]) if len(msg.position_covariance) > 0 else 1.0
            if not math.isfinite(cov_xy) or cov_xy <= 0.0:
                cov_xy = 1.0
        except Exception:
            cov_xy = 1.0
        for i in range(36):
            out.pose.covariance[i] = 0.0
        out.pose.covariance[0] = cov_xy
        out.pose.covariance[7] = cov_xy
        out.pose.covariance[14] = 0.5  # z
        out.pose.covariance[35] = (5.0 * math.pi / 180.0) ** 2  # yaw ~ 5 deg

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SingleGnssImuPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


