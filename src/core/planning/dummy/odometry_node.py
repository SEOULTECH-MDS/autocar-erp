#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to Quaternion (Z-W only)."""
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """Extract yaw angle from Quaternion."""
    return math.atan2(2.0 * quaternion.z * quaternion.w, 
                      1.0 - 2.0 * quaternion.z * quaternion.z)


class OdometryNode(Node):
    """Dead reckoning odometry node for path planning evaluation."""

    def __init__(self) -> None:
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('yaw0', 0.0)
        self.declare_parameter('rate', 20.0)  # Hz
        self.declare_parameter('publish_current_pose', True)
        self.declare_parameter('publish_tf', True)

        # Current state
        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.yaw = float(self.get_parameter('yaw0').value)
        self.v = 0.0  # linear velocity
        self.w = 0.0  # angular velocity

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        # TF broadcaster
        if bool(self.get_parameter('publish_tf').value):
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(PoseStamped, '/initial_pose', self._initial_pose_callback, 10)

        # Timer for odometry update
        self.last_time = self.get_clock().now()
        period = 1.0 / float(self.get_parameter('rate').value)
        self.timer = self.create_timer(period, self._update_odometry)

        self.get_logger().info(
            f'Odometry node started with initial pose: '
            f'x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}'
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle velocity commands from control node."""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _initial_pose_callback(self, msg: PoseStamped) -> None:
        """Handle initial pose from initial_pose_node."""
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'Initial pose frame_id is {msg.header.frame_id}, expected "map"'
            )
        
        # Reset position and orientation
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.orientation)
        
        self.get_logger().info(
            f'Reset to initial pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}'
        )

    def _update_odometry(self) -> None:
        """Update odometry using dead reckoning."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt > 0.0:
            # Dead reckoning integration
            # Update yaw first
            self.yaw += self.w * dt
            
            # Update position using current yaw
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position and orientation
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w
        
        self.odom_pub.publish(odom_msg)

        # Publish TF: odom -> base_link
        if bool(self.get_parameter('publish_tf').value):
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation = yaw_to_quaternion(self.yaw)
            
            self.tf_broadcaster.sendTransform(tf_msg)

        # Publish current pose in map frame (assuming map->odom is identity)
        if bool(self.get_parameter('publish_current_pose').value):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose = odom_msg.pose.pose
            
            self.current_pose_pub.publish(pose_msg)


def main() -> None:
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
