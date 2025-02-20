#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# gps 데이터 테스트용 노드
class hitech_test_pub(Node):
    def __init__(self):
        super().__init__('hitech_test_pub')
        self.publisher = self.create_publisher(NavSatFix, '/ublox_gps', 10)
        self.get_logger().info("hitech_test_pub START")

        self.create_timer(1/20, self.publish_gps)   # 20Hz 주기로 publish_gps 콜백 함수 호출

    # hitech 뒤쪽 좌표 를 GPS 메시지로 퍼블리싱
    def publish_gps(self):
        msg = NavSatFix()
        msg.latitude = 37.632010 #37.63187798
        msg.longitude = 127.076008 #127.07597029
        self.publisher.publish(msg)
        self.get_logger().info(f"GPS published: lat={msg.latitude}, lon={msg.longitude}")


def main(args=None):
    rclpy.init(args=args)
    node = hitech_test_pub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()