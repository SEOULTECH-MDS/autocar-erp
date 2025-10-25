#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# 장애물, 배달
class CameraFront(Node):
    def __init__(self):
        super().__init__('camera_front')

        self.publisher_ = self.create_publisher(Image, '/camera_front/image_raw', 10)
        self.camera_info_pub_ = self.create_publisher(CameraInfo, '/camera_front/camera_info', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다!")
        
        # Camera info 초기화
        self.camera_info_msg = self.create_camera_info()
        
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)  # 30Hz로 프레임 퍼블리시
        self.get_logger().info("camera_front 노드가 시작되었습니다.")

    def create_camera_info(self):
        """카메라 보정 정보 생성 (실제 캘리브레이션 값 사용)"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_front_link"
        camera_info.width = 640
        camera_info.height = 480
        
        # 카메라 내부 파라미터 (intrinsic matrix K)
        # 실제 카메라 보정을 통해 얻은 값
        camera_info.k = [
            612.44664,   0.0,     318.03385,
              0.0,     613.58285, 246.68416,
              0.0,       0.0,       1.0
        ]
        
        # 왜곡 계수 (distortion coefficients)
        # [k1, k2, t1, t2, k3]
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.061921, -0.176293, -0.005644, 0.002381, 0.0]
        
        # Rectification matrix
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix
        camera_info.p = [
            615.32024,   0.0,     319.22433,   0.0,
              0.0,     617.75832, 244.95193,   0.0,
              0.0,       0.0,       1.0,       0.0
        ]
        
        return camera_info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("카메라 프레임 읽기 실패")
            return
        
        # 현재 타임스탬프
        current_time = self.get_clock().now().to_msg()
        
        # 이미지 메시지 발행
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = current_time
        msg.header.frame_id = "camera_front_link"
        self.publisher_.publish(msg)
        
        # camera_info 메시지 발행
        self.camera_info_msg.header.stamp = current_time
        self.camera_info_pub_.publish(self.camera_info_msg)
        
        self.get_logger().debug("프레임 및 camera_info 퍼블리시 완료")

def main(args=None):
    rclpy.init(args=args)
    node = CameraFront()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("camera_front 노드가 종료됩니다.")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
