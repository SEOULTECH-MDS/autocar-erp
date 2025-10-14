#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class PoseTwistToOdom(Node):
    def __init__(self):
        super().__init__('pose_twist_to_odom')
        # Parameters
        self.declare_parameter('pose_topic', '/localization/pose_estimator/pose_with_covariance')
        self.declare_parameter('twist_topic', '/localization/twist_estimator/twist_with_covariance')
        self.declare_parameter('output_topic', '/autocar/location')

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.latest_pose: PoseWithCovarianceStamped | None = None
        self.latest_twist: TwistWithCovarianceStamped | None = None

        self.pub_odom = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, 10)
        self.create_subscription(TwistWithCovarianceStamped, twist_topic, self.on_twist, 10)

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg
        self.publish_if_ready()

    def on_twist(self, msg: TwistWithCovarianceStamped) -> None:
        self.latest_twist = msg
        self.publish_if_ready()

    def publish_if_ready(self) -> None:
        if self.latest_pose is None or self.latest_twist is None:
            return
        odom = Odometry()
        odom.header = self.latest_pose.header
        odom.child_frame_id = 'base_link'
        odom.pose = self.latest_pose.pose
        odom.twist = self.latest_twist.twist
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTwistToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


