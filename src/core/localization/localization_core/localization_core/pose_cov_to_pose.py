#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class PoseCovToPose(Node):
    def __init__(self):
        super().__init__('pose_cov_to_pose')
        self.declare_parameter('input_topic', '/localization/pose_selected')
        self.declare_parameter('output_topic', '/localization/pose_selected_pose')
        in_t = self.get_parameter('input_topic').get_parameter_value().string_value
        out_t = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(PoseStamped, out_t, 10)
        self.create_subscription(PoseWithCovarianceStamped, in_t, self.on_msg, 10)

    def on_msg(self, msg: PoseWithCovarianceStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseCovToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


