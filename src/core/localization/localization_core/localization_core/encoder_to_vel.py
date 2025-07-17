#!/usr/bin/env python3
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float32

OVERFLOW = 4294967296
OVERFLOW_2 = OVERFLOW//2

class EncoderVelPub(Node):
    def __init__(self):
        super().__init__('encoder_vel_pub')
        
        self.speed = Float32()
        self.prev_enc = 0.0
        self.prev_time = None
        self.enc_speed_data = deque(maxlen=5)       
        self.ser = serial.serial_for_url('/dev/ttyACM0', baudrate=9600, timeout=0.01)
        
        self.speed_pub = self.create_publisher(Float32, '/encoder_speed', 10)

    
    def get_value(self):
        enc_byte = self.ser.readline()
        
        enc_decoded = enc_byte.decode()
        
        if self.prev_time is None:
            self.prev_time = time.time()
            return
        
        cur_time = time.time()
        dt = cur_time - self.prev_time
        if dt < 0.05: # 20Hz
            return
        
        if not enc_decoded: # 빈 값이면
            self.get_logger().info('속도: 0 / error 1')
            vel = 0

        else:
            try:
                enc_first, enc_second = enc_decoded.split(',')
                # self.get_logger().info(enc_first, enc_second)
                
                # 엔코더 tick value
                cur_enc = (int(enc_first) - int(enc_second)) / 2
                self.get_logger().info('first', enc_first, 'second',enc_second)
                
                # 현재 tick - 이전 tick
                delta_enc = self.normalize_diff(cur_enc - self.prev_enc)
                self.get_logger().info('delta', delta_enc)
                
                # 속도 값으로 변환
                vel = delta_enc * 0.06283185307 * 0.265 /dt /4
                self.get_logger().info('현재 속도:', vel)
                
                self.prev_enc = cur_enc
            
            except:
                self.get_logger().info('속도: 0 / error 2')
                vel = 0
        
        # 평균값 필터
        self.enc_speed_data.append(vel)
        vel_filtered = np.mean(self.enc_speed_data)
            
        self.speed_pub.publish(vel_filtered)
    
        self.prev_time = cur_time
        # self.get_logger().info(enc_byte, ':', enc_decoded) 
        return
    
    def normalize_diff(self, diff):
        if diff > OVERFLOW_2:
            diff -= OVERFLOW
                
        elif diff < -OVERFLOW_2:
            diff += OVERFLOW
        
        return diff
    

def main(args=None):
    rclpy.init(args=args)
    encoder_vel_pub = EncoderVelPub()
    rclpy.spin(encoder_vel_pub)
    encoder_vel_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
