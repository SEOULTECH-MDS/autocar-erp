#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

from sensor_msgs.msg import PointCloud2


class PointCloudRestamper(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_restamper')

        # Parameters
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points/restamped')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Clocks for explicit stamping
        self._ros_clock = Clock(clock_type=ClockType.ROS_TIME)
        self._sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # IO
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.sub = self.create_subscription(PointCloud2, input_topic, self._callback, 10)

        self.get_logger().info(f"Restamping PointCloud2: '{input_topic}' -> '{output_topic}'")

    def _callback(self, msg: PointCloud2) -> None:
        try:
            use_sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except Exception:
            use_sim = False

        now_msg = (self._ros_clock.now() if use_sim else self._sys_clock.now()).to_msg()

        out_msg = copy.deepcopy(msg)
        out_msg.header.stamp = now_msg
        self.pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


