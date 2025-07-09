#!/usr/bin/env python3
# -*- coding: utf8 -*-

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from std_msgs.msg import String, Int32, Float32
from ackermann_msgs.msg import AckermannDriveStamped

from math import *
import numpy as np
import serial
import math
import time


def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('erp42_test')
    publisher = node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

    msg = AckermannDriveStamped()
    msg.drive.speed=0.0
    msg.drive.steering_angle = 0.261799 # desired virtual angle (radians) +20

    def timer_callback():
        msg.drive.steering_angle *= -1
        publisher.publish(msg)

    timer_period = 4  # seconds
    timer = node.create_timer(3, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


#def main(args=None):
#    rclpy.init(args=args)
#    pc_publisher = Ackermanntest()
#    rclpy.spin(pc_publisher)
#    pc_publisher.destroy_node()
#    rclpy.shutdown()

if __name__ == '__main__':
    main()
