#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
import serial
import sys

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)

    while not rclpy.is_shutdown():
        rclpy.init_node('yolo')
        ser_data = ser.read()
        if ser_data == "":
            print("=====404 NOT FOUND=====")
        else:
            print(ser_data)
