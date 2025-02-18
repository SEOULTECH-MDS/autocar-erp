#!/usr/bin/env python3
# -*- coding: utf8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

import time
from math import *
import numpy as np
import serial

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
gear = 0
speed = 0
steer = 0
brake = 0
count_alive=0
cur_ENC_backup=0

class erp42(Node):

	def __init__(self):
		super().__init__('delay')

		self.steer_angle = Float32()
		self.t = 0.0
		self.erp_steer = 0.0
		qos_profile = QoSProfile(depth=10)
		self.steer_pub = self.create_publisher(Float32, '/input_steer', qos_profile)
		self.steer_sub = self.create_subscription(Float32, '/output_steer', self.erp_steer_cb, qos_profile)

		self.ser = serial.serial_for_url("/dev/ttyUSB0", baudrate=115200, timeout=1)

		self.timer = self.create_timer(0.05, self.timer_callback)

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
		steer = np.rad2deg(steer*71) # rad/s to degree/s*71

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

	def timer_callback(self):
		speed = 0.0
		brake = 0
		gear = 0
		# steer = 0.5 * np.sin(self.t)
		# if abs(steer) > 0.45: steer = 0.45 * steer / abs(steer)
		# self.t += 0.2

		steer = np.radians(27)

		if self.t % 10 < 5: rev = 1
		else: rev = -1
		self.t += 1
		print(self.t, rev)
		steer = rev * steer
		self.steer_angle.data = steer
		self.steer_pub.publish(self.steer_angle)
		print("speed:",speed, "steer:", steer*180/np.pi, "brake",brake, "gear", gear)
		self.Send_to_ERP42(gear, speed, steer, brake)


		delay = self.calculate_delay(self.steer_angle, self.erp_steer)
		if delay is not None:
			print("딜레이: ", delay, " 초")
		else:
			print("딜레이를 찾을 수 없음")

	def erp_steer_cb(self, msg):
		self.erp_steer = msg.data


	def calculate_delay(self, input, output):
		start_time = time.time()  # 현재 시간 기록
		while True:
			current_time = time.time()  # 현재 시간 갱신
			elapsed_time = current_time - start_time  # 경과 시간 계산

			if output == input:  # 현재 조향각이 입력 조향각과 일치하면 딜레이를 반환
				return elapsed_time

			if elapsed_time >= 1.0:  # 경과 시간이 1초를 초과하면 딜레이를 찾을 수 없음
				return None

def main(args=None):
	rclpy.init(args=args)
	node = erp42()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
