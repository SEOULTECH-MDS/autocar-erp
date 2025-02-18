#!/usr/bin/env python3
# -*- coding: utf8 -*-

import os
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from autocar_msgs.msg import State2D
from ackermann_msgs.msg import AckermannDriveStamped

class Simul_Plot(Node):

	def __init__(self):
		super().__init__('simul_plot')
		self.ackermann_subscriber = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.acker_callback, 10)
		self.state_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_callback, 10)

		self.t = 0.0
		self.velocity = 0.0
		self.speed = 0.0
		self.cmd_steer = 0.0
		self.vision_steer = 0.0
		self.steer = 0.0
		self.brake = 0
		self.gear = 0
		self.dir = ['Forward', 'Forward', 'Backward']

		self.time = time.time()
		self.times = []
		self.target_value = []
		self.actual_value = []
		self.brake_value = []

		self.timer1 = self.create_timer(0.3, self.timer_callback)
		self.timer2 = self.create_timer(0.1, self.plot_creator)

	def real_steer(self, input_steer):
		input_range  = np.array([-22, -21, -18.5, -16, -13.5, -11,  -9.5,  -8,   -6, -4.5, -3, 0, 3, 4.5,   6,  8,  9.5, 11, 13.5, 16, 18.5, 21, 22])
		output_range = np.array([-27, -25, -22.5, -20, -17.5, -15, -12.5, -10, -7.5,   -5, -3, 0, 3,   5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 25, 27])

		output_steer = 0.0
		if input_steer >= max(input_range):
			output_steer = max(output_range)

		elif input_steer <= min(input_range):
			output_steer = min(output_range)

		else:
			output_steer = np.interp(input_steer, input_range, output_range)

		return np.deg2rad(output_steer)

	def faster_motor_control(self, target_speed, gps_vel):
		if target_speed != 0 and gps_vel <= 0.2:
			speed = 5.0
		else :
			speed = target_speed
		#print("target_speed", speed)

		return speed

	def vehicle_callback(self, msg):
		self.velocity = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
		if self.velocity >= 0.5:
			self.t += 0.1

	def acker_callback(self, msg):
		# self.speed = msg.drive.speed
		self.speed = self.faster_motor_control(msg.drive.speed, self.velocity)

		# self.steer = msg.drive.steering_angle
		cmd_steer = np.rad2deg(msg.drive.steering_angle)
		self.steer = self.real_steer(cmd_steer)
		# self.steer = self.vision_steer

		self.gear = int(msg.drive.acceleration)
   
		if self.t < 3:
			self.brake = 0
   
		elif self.velocity - self.speed > - 0.5:
			self.brake = int(msg.drive.jerk)
   
		else:
			self.brake = 0

	def timer_callback(self):
		# steer=radians(float(input("steer_angle:")))

		print("Speed :",round(self.speed*3.6, 1), "km/h\t Steer :", round(np.rad2deg(self.steer), 2), "deg\t Brake :",self.brake, "%\t Gear :", self.dir[self.gear])

	def plot_creator(self):
		elapsed_time = time.time() - self.time
		self.times.append(elapsed_time)
		self.target_value.append(self.speed * 3.6)
		self.actual_value.append(self.velocity * 3.6)
		self.brake_value.append(self.brake/10)

	def save(self, output_folder):
		count = 0
		output = os.path.join(output_folder, f'simul_plot_{count}.png')
		while os.path.exists(output):
				count += 1
				output = os.path.join(output_folder, f'simul_plot_{count}.png')

		# 그래프 그리기
		plt.figure(figsize=(10, 6))
		plt.plot(self.times, self.target_value, label='target_speed', color='blue')
		plt.plot(self.times, self.brake_value, label='brake_force', color='black')
		plt.xlabel("Time (s)")
		plt.ylabel("Speed (km/h)")
		plt.title("Speed Data Graph")
		plt.legend()

		plt.savefig(output)
		plt.show()

def main(args=None):
	rclpy.init(args=args)
	node = Simul_Plot()

	try:
		rclpy.spin(node)

	except KeyboardInterrupt:
		home_folder = os.path.expanduser("~")
		plot_folder = os.path.join(home_folder, 'dataset')
		node.save(plot_folder)
		node.get_logger().info('Plot Img Saved')

	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
