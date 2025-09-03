#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from tf2_ros import StaticTransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (rad) to a geometry_msgs/Quaternion (Z-W only)."""
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


class InitialPoseNode(Node):
    """Publish an initial pose for the planner and set static TF map->odom."""

    def __init__(self) -> None:
        super().__init__('initial_pose_node')

        # Parameters
        self.declare_parameter('x', 1.25)
        self.declare_parameter('y', 0)
        self.declare_parameter('yaw', math.pi/2)  # radians
        self.declare_parameter('publish_current_pose', False)  # optional keep-alive
        self.declare_parameter('rate', 5.0)  # Hz for keep-alive

        # Publishers
        self.initial_pose_pub = self.create_publisher(PoseStamped, '/initial_pose', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        # Static TF: map -> odom (identity)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([static_tf])

        # Publish initial pose once
        self._publish_initial_once()

        # Optional keep-alive publication for /current_pose until odometry is available
        if bool(self.get_parameter('publish_current_pose').value):
            period_sec = 1.0 / float(self.get_parameter('rate').value)
            self.timer = self.create_timer(period_sec, self._publish_current_pose)

    def _make_pose(self) -> PoseStamped:
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = yaw_to_quaternion(yaw)
        return pose_msg

    def _publish_initial_once(self) -> None:
        pose_msg = self._make_pose()
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"Initial pose published: x={pose_msg.pose.position.x:.2f}, "
            f"y={pose_msg.pose.position.y:.2f}"
        )

    def _publish_current_pose(self) -> None:
        pose_msg = self._make_pose()
        self.current_pose_pub.publish(pose_msg)


def main() -> None:
    rclpy.init()
    node = InitialPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


