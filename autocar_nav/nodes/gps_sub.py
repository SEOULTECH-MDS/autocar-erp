#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autocar_msgs.msg import State2D 
import pyproj


def latlon_to_utm(lat, lon):
    """
    위도(lat), 경도(lon)를 UTM 좌표로 변환6
    """
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)  # (utm_x, utm_y)


class GPS2UTM(Node):
    def __init__(self):
        super().__init__('gps_to_utm')

        # /ublox_gps 토픽을 구독 (NavSatFix 타입의 메시지)
        self.subscription = self.create_subscription(NavSatFix, '/ublox_gps', self.gps_callback, 10)

        # 변환된 UTM 좌표를 /autocar/state2D 토픽으로 퍼블리싱
        self.publisher = self.create_publisher(State2D, '/autocar/state2D', 10)

        self.get_logger().info("GPS to UTM START")

    def gps_callback(self, msg):
        """
        GPS 데이터를 받아 UTM 좌표로 변환 후 퍼블리싱
        """
        lat = msg.latitude
        lon = msg.longitude

        # UTM 변환 수행
        utm_x, utm_y = latlon_to_utm(lat, lon)

        # State2D 메시지 생성 및 데이터 설정
        state_msg = State2D()
        state_msg.pose.x = float(utm_x)
        state_msg.pose.y = float(utm_y)
        state_msg.pose.theta = 0.0  # 방향값이 없으므로 기본값 0 설정 (필요 시 수정)

        # 퍼블리싱
        self.publisher.publish(state_msg)

        self.get_logger().info(f"차량의 UTM 좌표: x={utm_x:.3f}, y={utm_y:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = GPS2UTM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
