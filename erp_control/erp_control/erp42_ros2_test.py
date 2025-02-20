#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

# ERP42 프로토콜 관련 상수
S = 0x53
T = 0x54
X = 0x58
AorM = 0x01
ESTOP = 0x00
GEAR = 0x00
SPEED0 = 0x00
SPEED1 = 0x00
STEER0 = 0x02
STEER1 = 0x02
BRAKE = 0x01
ALIVE = 0
ETX0 = 0x0d
ETX1 = 0x0a
count_alive = 0

class ERP42(Node):
    def __init__(self):
        super().__init__('erp42_node')

        # ROS2 서브스크라이버 (ERP42 제어 명령 수신)
        self.cmd_sub = self.create_subscription(
            AckermannDrive, '/erp_command', self.callback_cmd, 10)

        # Serial 통신 설정
        self.ser = serial.Serial("/dev/ttyERP", baudrate=115200, timeout=1)

        # 차량 상태 변수
        self.gear = 0  # 0: 전진, 2: 후진
        self.target_speed = 0.0
        self.target_steer = np.radians(0)
        self.target_brake = 1

        # 주기적으로 ERP42에 명령 전송
        self.timer_erp42 = self.create_timer(0.1, self.timer_callback_erp42)

    def callback_cmd(self, cmd_msg):
        """ /erp_command 토픽을 통해 명령을 수신하여 변수 업데이트 """
        self.target_speed = cmd_msg.speed
        self.target_brake = cmd_msg.jerk
        self.target_steer = np.radians(cmd_msg.steering_angle)

    def timer_callback_erp42(self):
        """ 주기적으로 ERP42에 명령 전송 """
        if self.target_speed < 0:
            self.gear = 2
            target_speed = -self.target_speed
        else:
            self.gear = 0
            target_speed = self.target_speed

        self.send_to_ERP42(self.gear, target_speed, self.target_steer, self.target_brake)

    def send_to_ERP42(self, gear, speed, steer, brake):
        """ ERP42 프로토콜에 맞게 데이터를 변환하여 전송 """
        global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive
        count_alive = (count_alive + 1) % 256  # 0xFF 이후 0으로 순환

        AorM = self.GetAorM()
        GEAR = self.GetGEAR(gear)
        SPEED0, SPEED1 = self.GetSPEED(speed)
        STEER0, STEER1 = self.GetSTEER(steer)
        BRAKE = self.GetBRAKE(brake)
        ALIVE = count_alive

        vals = [S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]
        self.ser.write(bytearray(vals))  # 바이트 배열 전송

        self.get_logger().info(f"Sent to ERP42: {vals}")
        self.get_logger().info(f"Gear: {gear}, Speed: {speed}, Brake: {brake}, Steer: {steer}")

    def GetGEAR(self, gear):
        return gear

    def GetSPEED(self, speed):
        """ 속도를 ERP42 프로토콜에 맞게 변환 """
        speed = max(0, min(5.0, speed))  # 속도 범위 제한
        speed_int = int(speed * 36)  # m/s -> km/h*10 변환

        SPEED0 = (speed_int >> 8) & 0xFF
        SPEED1 = speed_int & 0xFF
        return SPEED0, SPEED1

    def GetSTEER(self, steer):
        """ 조향각을 ERP42 프로토콜에 맞게 변환 """
        steer_deg = np.rad2deg(steer) * 71  # 라디안 -> ERP42 조향 입력값 변환
        
        # 최대 steer: 약 28도
        # 허용 범위 제한
        steer_int = max(-1999, min(1999, int(steer_deg)))

        if steer_int >= 0:
            STEER = 0b0000000000000000 + steer_int
        else:
            STEER = 0b1111100000110000 + (2000 - abs(steer_int))

        STEER0 = (STEER >> 8) & 0xFF
        STEER1 = STEER & 0xFF
        return STEER0, STEER1

    def GetBRAKE(self, brake):
        """ 브레이크 강도를 ERP42 프로토콜에 맞게 변환 """
        return min(200, abs(int(brake)))

    def GetAorM(self):
        return 0x01  # 자동 모드 (1)

def main(args=None):
    rclpy.init(args=args)
    erp42_node = ERP42()
    rclpy.spin(erp42_node)
    erp42_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
