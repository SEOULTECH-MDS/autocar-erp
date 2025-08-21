#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSign(Node):
    def __init__(self):
        super().__init__('camera_sign')
        
        self.publisher_ = self.create_publisher(Image, '/camera_sign/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다!")
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz로 프레임 퍼블리시
        self.get_logger().info("camera_sign 노드가 시작되었습니다.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("카메라 프레임 읽기 실패")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().debug("프레임 퍼블리시 완료")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("camera_sign 노드가 종료됩니다.")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()