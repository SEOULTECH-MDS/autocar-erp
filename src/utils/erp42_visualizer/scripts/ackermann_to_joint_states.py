#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import math

class StatePublisherNode(Node):

    def __init__(self):
        super().__init__('erp42_state_publisher')

        # --- Parameters ---
        self.wheel_radius = 0.28  # From URDF

        # --- State Variables ---
        self.steering_angle = 0.0
        self.wheel_rotation = 0.0
        self.target_speed = 0.0

        # --- ROS 2 Interfaces ---
        # Publisher for /joint_states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for /ackermann_cmd
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            '/erp/cmd_vel',
            self.ackermann_callback,
            10)

        # Timer to continuously update wheel rotation based on speed
        self.timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Ackermann to JointStates node has been started.')

    def ackermann_callback(self, msg):
        """
        Callback for AckermannDriveStamped messages.
        Updates steering and target speed.
        """
        self.steering_angle = msg.drive.steering_angle
        self.target_speed = msg.drive.speed
        # We don't call publish here directly to avoid flooding.
        # The timer will handle periodic publishing.

    def timer_callback(self):
        """
        Timer callback to update wheel rotation and publish joint states.
        """
        # Calculate wheel rotation increment based on speed and timer period
        # Angular velocity (rad/s) = Linear velocity (m/s) / Wheel radius (m)
        angular_velocity = self.target_speed / self.wheel_radius
        rotation_increment = angular_velocity * self.timer_period
        self.wheel_rotation += rotation_increment

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # These names MUST match the joint names in your URDF
        joint_state_msg.name = [
            'left_front_disk_wheel_joint', 'right_front_disk_wheel_joint',
            'left_front_wheel_joint', 'right_front_wheel_joint',
            'left_back_wheel_joint', 'right_back_wheel_joint'
        ]
        
        # Assign positions
        joint_state_msg.position = [
            self.steering_angle, self.steering_angle,
            self.wheel_rotation, self.wheel_rotation,
            self.wheel_rotation, self.wheel_rotation
        ]

        # Publish the message
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 