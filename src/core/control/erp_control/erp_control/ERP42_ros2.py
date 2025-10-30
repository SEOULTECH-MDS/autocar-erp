#!/usr/bin/env python3
# -*- coding: utf8 -*-

import os
import time
import serial
import numpy as np
import struct
try:
  import hid
except Exception:
  hid = None
try:
  import usb.core
  import usb.util
except Exception:
  usb = None

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

    self.ser = serial.Serial("/dev/ttyERP", baudrate=115200, timeout=1)
    self.rx_buffer = bytearray()
    self.last_aorm = None
    self.last_estop = None
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

    self.timer1 = self.create_timer(0.1, self.timer_callback)
    self.rx_timer = self.create_timer(0.02, self.serial_rx_timer)

    # Tower lamp parameters (HID)
    self.declare_parameter('enable_tower_lamp', True)
    self.declare_parameter('vid', 0x04d8)
    self.declare_parameter('pid', 0xe73c)
    self.declare_parameter('red_index', 0)
    self.declare_parameter('green_index', 2)
    self.declare_parameter('sound_on_value', 3)
    self.declare_parameter('sound_off_value', 0)
    self.declare_parameter('hid_report_id', 0)
    self.declare_parameter('hid_path', '')

    self.enable_tower_lamp = bool(self.get_parameter('enable_tower_lamp').get_parameter_value().bool_value)
    self.vid = int(self.get_parameter('vid').get_parameter_value().integer_value)
    self.pid = int(self.get_parameter('pid').get_parameter_value().integer_value)
    self.red_index = int(self.get_parameter('red_index').get_parameter_value().integer_value)
    self.green_index = int(self.get_parameter('green_index').get_parameter_value().integer_value)
    self.sound_on_value = int(self.get_parameter('sound_on_value').get_parameter_value().integer_value)
    self.sound_off_value = int(self.get_parameter('sound_off_value').get_parameter_value().integer_value)
    self.hid_report_id = int(self.get_parameter('hid_report_id').get_parameter_value().integer_value)
    self.hid_path = str(self.get_parameter('hid_path').get_parameter_value().string_value)

    # HID device handle and state
    self.hid_dev = None
    self.buzzer_toggle = False
    self.prev_lamp_payload = None
    # Tower lamp via pyusb fixed hex map
    self.declare_parameter('pad_len', 16)
    self.pad_len = int(self.get_parameter('pad_len').get_parameter_value().integer_value)
    self.usb_dev = None
    self.usb_ep_out = None
    self.usb_iface_num = None
    self._qlight_map = {
      'RED_ON': bytes.fromhex('57050164646464644000b0f000000000'),
      'RED_BLINK': bytes.fromhex('57050264646464644000b0f000000000'),
      'RED_OFF': bytes.fromhex('57050064646464644000b0f000000000'),
      'GREEN_ON': bytes.fromhex('57056464016464644000b0f000000000'),
      'GREEN_BLINK': bytes.fromhex('57056464026464644000b0f000000000'),
      'GREEN_OFF': bytes.fromhex('57056464006464644000b0f000000000'),
      'BUZZER_A': bytes.fromhex('57056464646464014000b0f000000000'),
      'BUZZER_OFF': bytes.fromhex('57056464646464004000f0f200000000'),
    }
    self._last_sent = (None, None, None)
    self._usb_open()

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
    SPEED = int(speed*36) # float to integer,  m/s to km/h*10
    SPEED1 = abs(SPEED) 
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

  # ===== Serial RX (PCU -> UPPER) parsing for AorM / ESTOP =====
  def serial_rx_timer(self):
    try:
      in_wait = self.ser.in_waiting
      if in_wait > 0:
        data = self.ser.read(in_wait)
        if data:
          self.rx_buffer.extend(data)

      # Try extracting as many frames as available
      while True:
        frame = self._extract_frame()
        if frame is None:
          break
        aorm, estop = self._parse_aorm_estop(frame)
        if aorm is not None and estop is not None:
          self.last_aorm = aorm
          self.last_estop = estop
          self._update_tower_lamp_mode()
    except Exception as e:
      self.get_logger().debug(f"serial_rx error: {e}")

  def _find_stx(self):
    buf = self.rx_buffer
    for i in range(max(0, len(buf) - 21) if len(buf) > 21 else 0, len(buf)):
      if i + 2 < len(buf) and buf[i] == 0x53 and buf[i+1] == 0x54 and buf[i+2] == 0x58:
        return i
    return None

  def _extract_frame(self):
    FRAME_LEN = 18
    if len(self.rx_buffer) < FRAME_LEN:
      return None
    stx_idx = self._find_stx()
    if stx_idx is None:
      drop_len = max(0, len(self.rx_buffer) - 2)
      if drop_len:
        del self.rx_buffer[:drop_len]
      return None
    if stx_idx + FRAME_LEN > len(self.rx_buffer):
      if stx_idx > 0:
        del self.rx_buffer[:stx_idx]
      return None
    frame = bytes(self.rx_buffer[stx_idx:stx_idx+FRAME_LEN])
    if frame[-2] != 0x0D or frame[-1] != 0x0A:
      del self.rx_buffer[:stx_idx+1]
      return None
    del self.rx_buffer[:stx_idx+FRAME_LEN]
    return frame

  def _parse_aorm_estop(self, frame):
    try:
      # Indices: 3=AorM, 4=ESTOP (PCU->UPPER, little endian only affects multi-byte fields)
      aorm = frame[3]
      estop = frame[4]
      return aorm, estop
    except Exception:
      return None, None

  # ===== Tower Lamp (HID) control =====
  def _ensure_hid(self):
    if not self.enable_tower_lamp:
      return False
    if hid is None:
      self.get_logger().throttle(5000, f"hidapi not available; install 'hidapi' to enable tower lamp")
      return False
    if self.hid_dev is not None:
      return True
    # Try multiple fallback strategies
    try:
      if self.hid_path:
        self._hid_open_by_path(self.hid_path)
        return True
      self._hid_open_by_vid_pid()
      return True
    except Exception as e:
      # enumerate and retry
      paths = self._hid_enumerate_paths(self.vid, self.pid)
      if paths:
        try:
          self._hid_open_by_path(paths[0])
          return True
        except Exception as e2:
          self.get_logger().throttle(5000, f"HID open failed by path: {e2}")
      self.get_logger().throttle(5000, f"HID open failed (vid=0x{self.vid:04x}, pid=0x{self.pid:04x}): {e}")
      return False

  def _hid_open_by_vid_pid(self):
    if hasattr(hid, 'Device'):
      self.hid_dev = hid.Device(vid=self.vid, pid=self.pid)
    else:
      d = hid.device()
      d.open(self.vid, self.pid)
      self.hid_dev = d

  def _hid_open_by_path(self, path: str):
    if hasattr(hid, 'Device'):
      self.hid_dev = hid.Device(path=path.encode() if isinstance(path, str) else path)
    else:
      d = hid.device()
      d.open_path(path.encode() if isinstance(path, str) else path)
      self.hid_dev = d

  def _hid_enumerate_paths(self, vid: int, pid: int):
    try:
      infos = hid.enumerate(vid, pid)
      paths = []
      for info in infos:
        p = info.get('path') if isinstance(info, dict) else getattr(info, 'path', None)
        if p:
          if isinstance(p, bytes):
            p = p.decode(errors='ignore')
          paths.append(p)
      return paths
    except Exception:
      return []

  # ===== pyusb + fixed hex map control =====
  def _usb_open(self):
    if not self.enable_tower_lamp:
      return
    if usb is None:
      self.get_logger().throttle(5000, "pyusb가 필요합니다. 'sudo apt install python3-usb' 또는 'pip install pyusb'")
      return
    try:
      dev = usb.core.find(idVendor=self.vid, idProduct=self.pid)
      if dev is None:
        self.get_logger().throttle(5000, "USB 장치(타워램프)를 찾지 못했습니다")
        return
      try:
        if dev.is_kernel_driver_active(0):
          try:
            dev.detach_kernel_driver(0)
          except Exception:
            pass
      except Exception:
        pass
      try:
        dev.set_configuration()
      except Exception:
        pass
      ep_out = None
      iface_num = None
      for cfg in dev:
        for intf in cfg:
          if intf.bInterfaceClass == 3:
            for ep in intf:
              if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_OUT:
                ep_out = ep.bEndpointAddress
                iface_num = intf.bInterfaceNumber
                break
          if ep_out is not None:
            break
        if ep_out is not None:
          break
      if ep_out is None:
        self.get_logger().throttle(5000, "OUT endpoint(0x01)를 찾지 못했습니다")
        return
      usb.util.claim_interface(dev, iface_num)
      self.usb_dev = dev
      self.usb_ep_out = ep_out
      self.usb_iface_num = iface_num
      self.get_logger().info(f"pyusb open OK iface={iface_num} ep_out=0x{ep_out:02x}")
    except Exception as e:
      self.get_logger().throttle(5000, f"pyusb open 실패: {e}")

  def _send_known(self, key: str):
    if not self.enable_tower_lamp or self.usb_dev is None:
      return
    data = self._qlight_map.get(key)
    if not data:
      return
    out = data if len(data) >= self.pad_len else data + bytes([0x00] * (self.pad_len - len(data)))
    try:
      self.usb_dev.write(self.usb_ep_out, out, timeout=200)
    except Exception as e:
      self.get_logger().debug(f"usb write 실패: {e}")

  def _update_tower_lamp_mode(self):
    # Estop 모드: 초록 OFF, 빨강 ON, 부저 OFF
    # Auto 모드: 초록 BLINK, 빨강 OFF, 부저 ON
    # Manual 모드: 초록 OFF, 빨강 OFF, 부저 OFF
    if not self.enable_tower_lamp:
      return
    if self.last_estop == 1:
      desired = ('RED_ON', 'GREEN_OFF', 'BUZZER_OFF')
    elif self.last_aorm == 0:
      desired = ('RED_OFF', 'GREEN_OFF', 'BUZZER_OFF')
    elif self.last_aorm == 1:
      desired = ('RED_OFF', 'GREEN_BLINK', 'BUZZER_A')
    else:
      return

    if desired == self._last_sent:
      return
    red_key, green_key, buzzer_key = desired
    self._send_known(red_key)
    self._send_known(green_key)
    self._send_known(buzzer_key)
    self._last_sent = desired

  def buzzer_timer_callback(self):
    # Toggle only when Auto mode and not ESTOP
    if self.last_aorm == 0 and (self.last_estop is None or self.last_estop == 0):
      self.buzzer_toggle = not self.buzzer_toggle
      self._update_tower_lamp(force_send=False)
    else:
      # Ensure buzzer off in other states
      if self.buzzer_toggle:
        self.buzzer_toggle = False
        self._update_tower_lamp(force_send=False)

  def _update_tower_lamp(self, force_send=True):
    if not self._ensure_hid():
      return

    # Build lamp payload [lamp0..lamp4, sound]
    lamp = [100, 100, 100, 100, 100]
    sound = self.sound_off_value

    if self.last_estop == 1:
      # ESTOP 우선: Red ON, Green OFF, buzzer OFF
      lamp[self.red_index] = 1
      if 0 <= self.green_index < 5:
        lamp[self.green_index] = 0
      sound = self.sound_off_value
    elif self.last_aorm == 0:
      # Auto 모드: Green ON, Red OFF, buzzer 1s 토글
      if 0 <= self.red_index < 5:
        lamp[self.red_index] = 0
      lamp[self.green_index] = 1
      sound = self.sound_on_value if self.buzzer_toggle else self.sound_off_value
    else:
      # Others: all off
      lamp = [0, 0, 0, 0, 0]
      sound = self.sound_off_value

    payload = bytes(lamp + [sound])
    if not force_send and self.prev_lamp_payload == payload:
      return
    self.prev_lamp_payload = payload

    # Try send with report ID 0 prefix first, then without
    try:
      report_id = self.hid_report_id
      if report_id == 0:
        out = bytes([0x00]) + payload
      else:
        out = bytes([report_id]) + payload
      self.hid_dev.write(out)
    except Exception:
      try:
        self.hid_dev.write(payload)
      except Exception as e:
        self.get_logger().debug(f"HID write failed: {e}")

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
    
    # 속도가 음수이면 후진 기어(2)로 설정, 양수이면 전진 기어(0)로 설정
    if msg.drive.speed < 0:
        self.gear = 2  # 후진
        self.speed = abs(msg.drive.speed)  # 속도는 절댓값으로 사용
    else:
        self.gear = int(msg.drive.acceleration) if msg.drive.acceleration in [0, 1, 2] else 0  # 전진 또는 기존 로직
        self.speed = msg.drive.speed
    
    # target speed 0일때 급정지
    if msg.drive.speed == 0.0:
      self.speed = 0.0
      self.steer = 0.0
      self.brake = 200
      return

    cmd_steer = np.rad2deg(msg.drive.steering_angle)
    self.steer = self.real_steer(cmd_steer)
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
        self.speed = abs(msg.drive.speed)  # 후진일 경우를 고려하여 절댓값 사용

    else:
      # 단순하게 목표 속도 그대로 사용
      self.speed = abs(msg.drive.speed)
      self.brake = 1


  def timer_callback(self):
    # steer=radians(float(input("steer_angle:")))

    print("Speed :", round(self.speed*3.6, 1), " km/h\t", "Steer :", round(np.rad2deg(self.steer), 2), " deg\t",
          "Brake :", self.brake, " %\t", 									"Gear :", self.dir[self.gear])
    self.Send_to_ERP42(self.gear, self.speed, -self.steer, self.brake)

def main(args=None):
  rclpy.init(args=args)
  node = erp42()

  try:
    rclpy.spin(node)

  except KeyboardInterrupt:
    node.get_logger().info('Node stopped')

  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()