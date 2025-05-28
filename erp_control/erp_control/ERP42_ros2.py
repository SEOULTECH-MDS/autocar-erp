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
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

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
    self.ackermann_subscriber = self.create_subscription(AckermannDriveStamped, '/erp/cmd_vel', self.acker_callback, 10)
    self.state_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_callback, 10)
    # self.state_sub = self.create_subscription(Odometry, '/data/encoder_vel_two', self.vehicle_callback, 10)
    self.ser = serial.Serial("/dev/ttyERP", baudrate=115200, timeout=1)
    self.departure = time.time()
    self.target_speed = 0.0
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

    ## PID const
    self.kp = 1.7
    self.ki = 0
    self.kd = 0.0
    self.prev_error = 0
    self.integral = 0

    # plot variable
    self.time = time.time()
    self.times = []
    self.target_value = []
    self.actual_value = []
    self.brake_value = []
    self.input_value = []

    self.timer1 = self.create_timer(0.1, self.timer_callback)
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
    vals = [S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]
    #self.ser.write(bytearray(vals))  # 바이트 배열 전송
    print(vals[8].to_bytes(1, byteorder='big'),vals[9].to_bytes(1, byteorder='big'))
    for i in range(len(vals)):
      self.ser.write(vals[i].to_bytes(1, byteorder='big')) # send!
      
    self.get_logger().info(f"Sent to ERP42: {vals}")
    self.get_logger().info(f"Gear: {gear}, Speed: {speed}, Brake: {brake}, Steer: {steer}")

    # vals = [S, T, X, AorM, ESTOP,GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]
    # # self.ser.write(bytearray(vals))
    # # print(vals[8], vals[9])
    # # print(hex(vals[8]), hex(vals[9]))
    # # print(vals[8].to_bytes(1, byteorder='big'),vals[9].to_bytes(1, byteorder='big'))
    # print(vals[8], vals[9])
    # print(hex(vals[8]), hex(vals[9]))


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

  def vehicle_callback(self, msg):
    self.velocity = np.sqrt((msg.twist.twist.linear.x**2.0) + (msg.twist.twist.linear.y**2.0))
    # self.velocity = msg.twist.twist.linear.x
    if self.velocity >= 0.5:
      self.departure += 0.1

  def speed_control(self, target_speed):
    self.speed = 0
    self.brake = 65


  def acker_callback(self, msg):
    self.target_speed = msg.drive.speed
    # target speed 0일때 급정지
    if msg.drive.speed == 0.0:
      self.speed = 0.0
      self.steer = 0.0
      self.brake = 200
      return

    cmd_steer = np.rad2deg(msg.drive.steering_angle)
    self.steer = self.real_steer(cmd_steer)
    self.gear = int(msg.drive.acceleration)
    parking = bool(msg.drive.jerk)

    if parking:
      self.brake = 1
      if msg.drive.speed == 0.0:
        self.speed = 0.0
        self.steer = 0.0
        self.brake = 200 if self.velocity > 1e-6 else 0
      elif self.velocity < 0.5:
        self.speed = 15/3.6
      else:
        self.speed = msg.drive.speed

    else:
      if self.velocity - msg.drive.speed >= 2.5/3.6 : # 0.42 : 1.5km/h,  0.28 : 1km/h
        self.speed = msg.drive.speed
        self.brake = 57
      elif self.velocity - msg.drive.speed < -3/3.6:  # msg.drive.speed <= 10 and
        if msg.drive.speed < 8/3.6:
          self.speed = msg.drive.speed + 4/3.6
        else:
          self.speed = msg.drive.speed + 2/3.6
        self.brake = 1
      else:
        self.speed = msg.drive.speed
        self.brake = 1


  def timer_callback(self):
    # steer=radians(float(input("steer_angle:")))

    print("Speed :", round(self.speed*3.6, 1), " km/h\t", "Steer :", round(np.rad2deg(self.steer), 2), " deg\t",
          "Brake :", self.brake, " %\t", 									"Gear :", self.dir[self.gear])
    self.Send_to_ERP42(self.gear, self.speed, -self.steer, self.brake)

  def plot_creator(self):
    elapsed_time = time.time() - self.time
    self.times.append(elapsed_time)
    self.target_value.append(self.target_speed * 3.6)
    self.input_value.append(self.speed * 3.6)
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
    plt.plot(self.times, self.input_value, label='input', color='yellow')
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