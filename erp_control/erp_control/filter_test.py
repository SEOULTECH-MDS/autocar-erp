#!/usr/bin/env python3
# -*- coding: utf8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

import time
import numpy as np

class LowPassFilter:
    def __init__(self, cutoff_freq, update_rate):
        self.update_rate = update_rate
        self.alpha = cutoff_freq / (cutoff_freq + update_rate)
        self.filtered_angle = 0.0

    def update(self, input_angle):
        self.filtered_angle += self.alpha * (input_angle - self.filtered_angle)

        return self.filtered_angle


class erp42(Node):

	def __init__(self):
		super().__init__('test')

		self.steer_angle = Float32()
		self.t = 0.0
		self.rev = 1
		self.erp_steer = 0.0
		qos_profile = QoSProfile(depth=10)
		self.input_pub = self.create_publisher(Float32, '/input_steer', qos_profile)
		self.output_pub = self.create_publisher(Float32, '/output_steer', qos_profile)

		self.filter = LowPassFilter(cutoff_freq=2.0, update_rate=10.0)
		self.arrived = False
		self.input_time = time.time()
		self.arrival_time = time.time()

		self.timer1 = self.create_timer(0.1, self.input_callback)
		self.timer2 = self.create_timer(0.1, self.output_callback)

	def input_callback(self):
		steer = np.radians(22)

		if self.t % 100 < 50: rev = 1
		else: rev = -1
		self.t += 1

		if self.rev != rev:
			self.arrived = True
			self.input_time = time.time()
		self.rev = rev

		self.steer_angle.data = rev * steer
		self.input_pub.publish(self.steer_angle)

	def output_callback(self):
		output = Float32()
		output.data = self.filter.update(self.steer_angle.data)
		self.output_pub.publish(output)

		if self.arrived:
			if abs(output.data - self.steer_angle.data) <= 1e-2:
				self.arrival_time = time.time()
				delay = self.arrival_time - self.input_time

				print(f'도달 시간 : {delay:.2f} 초')

				self.arrived = False



def main(args=None):
	rclpy.init(args=args)
	node = erp42()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
