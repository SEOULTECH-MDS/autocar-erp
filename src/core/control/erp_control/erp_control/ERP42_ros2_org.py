#!/usr/bin/env python3
# -*- coding: utf8 -*-

import os
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from autocar_msgs.msg import State2D
from ackermann_msgs.msg import AckermannDriveStamped

fast_flag =False
S = 0x53
T = 0x54
X = 0x58
AorM = 0x01
ESTOP = 0x00
GEAR = 0x00
SPEED0 = 0x00
SPEED1 = 0x00
STEER0 = 0X02
STEER1 = 0x02
BRAKE = 0x01
ALIVE = 0
ETX0 = 0x0d
ETX1 = 0x0a
Packet=[]
read=[]
count=0
count_alive=0
cur_ENC_backup=0

class erp42(Node):

	def __init__(self):
		super().__init__('erp42')
		self.ackermann_subscriber = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.acker_callback, 10)
		self.state_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_callback, 10)
		self.ser = serial.serial_for_url("/dev/ttyERP", baudrate=115200, timeout=1)

		self.departure = time.time()
		self.velocity = 0.0
		self.speed = 0.0
		self.cmd_steer = 0.0
		self.vision_steer = 0.0
		self.steer = 0.0
		self.brake = 0
		self.gear = 0
		self.dir = ['Forward', 'Forward', 'Backward']

		self.prev_speed = 0.0
		self.prev_gear = 0
		self.gear_change = False
		self.slow_down = False
		self.brake_time = time.time()
		self.brake_force = 0
		self.t = 0
		self.dt = 0.3

		# plot variable
		self.time = time.time()
		self.times = []
		self.target_value = []
		self.actual_value = []
		self.brake_value = []

		self.timer1 = self.create_timer(0.3, self.timer_callback)
		self.timer2 = self.create_timer(0.1, self.plot_creator)

	def GetAorM(self):
		AorM = 0x01
		return  AorM

	def GetESTOP(self):
		ESTOP = 0x00
		return  ESTOP

	def GetGEAR(self, gear):
		GEAR = gear
		return  GEAR

	def GetSPEED(self, speed):
		global count
		SPEED0 = 0x00
		SPEED = int(speed*36) # float to integer
		SPEED1 = abs(SPEED) # m/s to km/h*10
		return SPEED0, SPEED1

	def GetSTEER(self, steer): # steer은 rad/s 값으로 넣어줘야한다.
		steer=steer*71*(180/np.pi) # rad/s to degree/s*71

		if(steer>=2000):
			steer=1999
		elif(steer<=-2000):
			steer=-1999
		steer_max=0b0000011111010000 # +2000
		steer_0 = 0b0000000000000000
		steer_min=0b1111100000110000 # -2000

		if (steer>=0):
			angle=int(steer)
			STEER=steer_0+angle
		else:
			angle=int(-steer)
			angle=2000-angle
			STEER=steer_min+angle

		STEER0=STEER & 0b1111111100000000
		STEER0=STEER0 >> 8
		STEER1=STEER & 0b0000000011111111
		return STEER0, STEER1

	def GetBRAKE(self, brake):
		BRAKE = brake
		return  BRAKE

	def Send_to_ERP42(self, gear, speed, steer, brake):
		global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive
		count_alive = count_alive+1

		if count_alive==0xff:
			count_alive=0x00

		AorM = self.GetAorM()
		ESTOP = self.GetESTOP()
		GEAR = self.GetGEAR(gear)
		SPEED0, SPEED1 = self.GetSPEED(speed)
		STEER0, STEER1 = self.GetSTEER(steer)
		BRAKE = self.GetBRAKE(brake)

		ALIVE = count_alive

		vals = [S, T, X, AorM, ESTOP,GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]
		# print(vals[8], vals[9])
		# print(hex(vals[8]), hex(vals[9]))
		# print(vals[8].to_bytes(1, byteorder='big'),vals[9].to_bytes(1, byteorder='big'))
		for i in range(len(vals)):
			self.ser.write(vals[i].to_bytes(1, byteorder='big')) # send!

		# for i in range(8, 10):
		# 	self.ser.write(vals[i].to_bytes(1, byteorder='big')) # send!

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
			self.departure += 0.1

	def acker_callback1(self, msg):
		# self.speed = msg.drive.speed
		self.speed = self.faster_motor_control(msg.drive.speed, self.velocity)

		# self.steer = msg.drive.steering_angle
		cmd_steer = np.rad2deg(msg.drive.steering_angle)
		self.steer = self.real_steer(cmd_steer)
		# self.steer = self.vision_steer

		self.gear = int(msg.drive.acceleration)

		# self.brake = int(msg.drive.jerk)
		if self.departure < 5:
			self.brake = 0

		elif self.velocity - self.speed > - 0.5:
			self.brake = int(msg.drive.jerk)

		else:
			self.brake = 0

	def brake_control(self, b, m, t=3):
		brake = b * self.t
		self.t += self.dt

		if brake >= m: brake = m

		return int(brake)

	def acker_callback(self, msg):
		# self.steer = msg.drive.steering_angle
		cmd_steer = np.rad2deg(msg.drive.steering_angle)
		self.steer = self.real_steer(cmd_steer)

		quick_stop = bool(msg.drive.jerk)

		input_speed = msg.drive.speed
		if self.prev_speed - input_speed > 0 and input_speed != 0:
			if self.velocity - self.prev_speed > -0.5 and self.velocity > 8/3.6:
				if not self.slow_down:
					self.brake_time = time.time()
					self.brake_force = 100 * (self.prev_speed - input_speed) / self.prev_speed
				self.slow_down = True

		self.gear = int(msg.drive.acceleration)
		if self.gear != self.prev_gear:
			self.gear_change = True

		# Full brake
		if quick_stop:
			print("Quick Stop")
			self.speed = 0.0

			if self.velocity > 0.1:
				self.brake_time = time.time()

			if time.time() - self.brake_time < 2:
				self.steer = 0.0
				self.brake = 200

		# Gear Change
		elif self.gear_change:
			print("Gear Change")
			self.speed = 0.0

			if self.velocity > 0.1:
				self.brake_time = time.time()

			if time.time() - self.brake_time < 1:
				self.steer = 0.0
				self.brake = 200

			else:
				self.gear_change = False
				self.brake = 0

		# Deceleration zones
		elif self.slow_down:
			print("Slow Down")

			if self.velocity - input_speed > -0.5:
				self.speed = 0.0
				self.brake = int(self.brake_force)

			if time.time() - self.brake_time > 5 or self.velocity < input_speed - 0.5:
				self.slow_down = False
				self.brake = 0
				self.t = 0

		else:
			print("Else")
			self.speed = self.faster_motor_control(input_speed, self.velocity)
			self.brake = 0

		# On initial departure
		if self.departure < 5:
			self.brake = 0

		self.prev_gear = self.gear
		self.prev_speed = input_speed

	def timer_callback(self):
		# steer=radians(float(input("steer_angle:")))

		print("Speed :", round(self.speed*3.6, 1), "km/h\t", "Steer :", round(np.rad2deg(self.steer), 2), "deg\t",
					"Brake :", self.brake, "%\t", 									"Gear :", self.dir[self.gear])
		self.Send_to_ERP42(int(self.gear), float(self.speed), -float(self.steer), int(self.brake))

	def plot_creator(self):
		elapsed_time = time.time() - self.time
		self.times.append(elapsed_time)
		self.target_value.append(self.speed * 3.6)
		self.actual_value.append(self.velocity * 3.6)
		self.brake_value.append(self.brake/10)

	def save(self, output_folder):
		count = 0
		output = os.path.join(output_folder, f'speed_graph_{count}.png')
		while os.path.exists(output):
				count += 1
				output = os.path.join(output_folder, f'speed_graph_{count}.png')

		# 그래프 그리기
		plt.figure(figsize=(10, 6))
		plt.plot(self.times, self.target_value, label='target_speed', color='blue')
		plt.plot(self.times, self.actual_value, label='actual_speed', color='red')
		plt.plot(self.times, self.brake_value, label='brake_force', color='black')
		plt.xlabel("Time (s)")
		plt.ylabel("Speed (km/h)")
		plt.title("Speed Data Graph")
		plt.legend()

		plt.savefig(output)
		plt.show()

def main(args=None):
	rclpy.init(args=args)
	node = erp42()

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
