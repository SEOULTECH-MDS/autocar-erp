#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import math
from rclpy.clock import Clock, ClockType

class StatePublisherNode(Node):

    def __init__(self):
        super().__init__('erp42_state_publisher')

        # --- Parameters ---
        self.wheel_radius = 0.28  # From URDF

        # Clocks for explicit stamping
        self._ros_clock = Clock(clock_type=ClockType.ROS_TIME)
        self._sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # --- State Variables ---
        self.steering_angle = 0.0
        self.wheel_rotation = 0.0
        self.target_speed = 0.0

        # --- ROS 2 Interfaces ---
        # Publisher for /joint_states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

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
        # If the target speed is not zero, rotate the wheels at a constant speed.
        # Otherwise, the wheels will remain stationary.
        if self.target_speed != 0.0:
            constant_rotation_increment = 0.1  # Constant value for visual rotation
            self.wheel_rotation += constant_rotation_increment
            self.wheel_rotation %= (2 * math.pi) # Normalize the angle to [0, 2*pi]

        # Create JointState message
        joint_state_msg = JointState()
        # Explicitly choose ROS time or system time based on use_sim_time
        try:
            use_sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except Exception:
            use_sim = False
        now_msg = (self._ros_clock.now() if use_sim else self._sys_clock.now()).to_msg()
        joint_state_msg.header.stamp = now_msg
        
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