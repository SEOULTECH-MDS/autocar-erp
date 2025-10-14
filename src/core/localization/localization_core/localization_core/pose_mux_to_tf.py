#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time


def time_to_float(t: Time) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


class PoseMuxToTF(Node):
    def __init__(self):
        super().__init__('pose_mux_to_tf')
        # Parameters
        self.declare_parameter('yabloc_pose_topic', '/localization/pose_estimator/pose_with_covariance')
        self.declare_parameter('gnss_pose_topic', '/sensing/gnss/pose_with_covariance')
        self.declare_parameter('selected_pose_topic', '/localization/pose_selected')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('yabloc_timeout_sec', 0.5)
        self.declare_parameter('publish_tf', True)
        # Fusion thresholds
        self.declare_parameter('max_pos_std_m', 0.7)   # sqrt(cov_x), sqrt(cov_y)
        self.declare_parameter('max_yaw_std_deg', 8.0) # sqrt(cov_yaw)
        self.declare_parameter('ekf_timeout_sec', 0.5)

        yabloc_topic = self.get_parameter('yabloc_pose_topic').get_parameter_value().string_value
        gnss_topic = self.get_parameter('gnss_pose_topic').get_parameter_value().string_value
        self.selected_topic = self.get_parameter('selected_pose_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.yabloc_timeout = float(self.get_parameter('yabloc_timeout_sec').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_selected = self.create_publisher(PoseWithCovarianceStamped, self.selected_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, yabloc_topic, self.on_yabloc, 10)
        self.create_subscription(PoseWithCovarianceStamped, gnss_topic, self.on_gnss, 10)
        # Optional EKF source
        self.declare_parameter('ekf_pose_topic', '/localization/ekf/pose_with_covariance')
        ekf_topic = self.get_parameter('ekf_pose_topic').get_parameter_value().string_value
        self.create_subscription(PoseWithCovarianceStamped, ekf_topic, self.on_ekf, 10)
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)

        self.last_yabloc: PoseWithCovarianceStamped | None = None
        self.last_gnss: PoseWithCovarianceStamped | None = None
        self.last_ekf: PoseWithCovarianceStamped | None = None

        self.timer = self.create_timer(0.02, self.tick)

    def on_yabloc(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_yabloc = msg

    def on_gnss(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_gnss = msg

    def on_ekf(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_ekf = msg

    def tick(self) -> None:
        now = self.get_clock().now().to_msg()
        now_f = time_to_float(now)

        selected: PoseWithCovarianceStamped | None = None

        # 1) Prefer YabLoc if fresh and precise
        if self.last_yabloc is not None:
            t = time_to_float(self.last_yabloc.header.stamp)
            if now_f - t <= self.yabloc_timeout:
                if self._is_precise(self.last_yabloc):
                    selected = self.last_yabloc

        # 2) Else EKF if fresh
        if selected is None and self.last_ekf is not None:
            t = time_to_float(self.last_ekf.header.stamp)
            if now_f - t <= float(self.get_parameter('ekf_timeout_sec').value):
                selected = self.last_ekf

        # 3) Else fall back to GNSS
        if selected is None and self.last_gnss is not None:
            selected = self.last_gnss

        if selected is None:
            return

        # publish selected pose
        self.pub_selected.publish(selected)

        # publish TF (optional)
        if self.publish_tf_enabled:
            tmsg = TransformStamped()
            tmsg.header.stamp = selected.header.stamp
            tmsg.header.frame_id = self.parent_frame
            tmsg.child_frame_id = self.child_frame
            tmsg.transform.translation.x = selected.pose.pose.position.x
            tmsg.transform.translation.y = selected.pose.pose.position.y
            tmsg.transform.translation.z = selected.pose.pose.position.z
            tmsg.transform.rotation = selected.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tmsg)

    def _is_precise(self, msg: PoseWithCovarianceStamped) -> bool:
        cov = msg.pose.covariance
        if len(cov) != 36:
            return False
        try:
            std_x = (cov[0]) ** 0.5
            std_y = (cov[7]) ** 0.5
            std_yaw = (cov[35]) ** 0.5
        except Exception:
            return False
        return (std_x <= float(self.get_parameter('max_pos_std_m').value) and
                std_y <= float(self.get_parameter('max_pos_std_m').value) and
                std_yaw <= float(self.get_parameter('max_yaw_std_deg').value) * 3.1415926535 / 180.0)


def main(args=None):
    rclpy.init(args=args)
    node = PoseMuxToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


