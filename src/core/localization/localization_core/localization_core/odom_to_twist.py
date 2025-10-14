#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped


class OdomToTwist(Node):
    def __init__(self):
        super().__init__('odom_to_twist')

        # Parameters
        self.declare_parameter('odom_topic', '/autocar/location')
        self.declare_parameter('output_topic', '/localization/twist_estimator/twist_with_covariance')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Publisher / Subscriber
        self.pub_twist = self.create_publisher(TwistWithCovarianceStamped, output_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)

    def on_odom(self, msg: Odometry) -> None:
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.twist = msg.twist
        self.pub_twist.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


