#!/usr/bin/env python3

# ✓ get_package_share_directory 임포트 (패키지 리소스 경로 획득)
from ament_index_python.packages import get_package_share_directory
import os

import numpy as np
from cv_bridge import CvBridge
import cv2
import traceback  # ✓ 예외 출력용 traceback 임포트

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String, Float32
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from perception.lanenet.ultrafastLaneDetector import UltrafastLaneDetector, ModelType
from geometry_msgs.msg import Point
from scipy.stats import iqr

class LaneNet(Node):

    def __init__(self):

        super().__init__('lanenet')

        self.image_sub = self.create_subscription(Image, "/lane/image_raw", self.image_cb, 10)

        self.steer_pub = self.create_publisher(Float64MultiArray, "/lanenet_steer", 10)
        self.image_pub = self.create_publisher(Image, "/lanenet_image", 10)
        self.lanent_lane_pub = self.create_publisher(Marker, '/rviz/lanenet_lane', 10)
        self.lateral_error_pub = self.create_publisher(Float32, '/lanenet/lateral_error', 10)

        self.lateral_error = Float32()
        
        self.steer_angle = Float64MultiArray()
        self.K = 0.1

        # model_path = 'models/culane_18.pth'
        package_share_dir = get_package_share_directory('perception')
        model_path = os.path.join(package_share_dir, 'models', 'culane_18.pth')
        model_type = ModelType.CULANE
        use_gpu = True
        self.reliability_var = 1000

        self.lateral_error_array = []

        # ✓ Initialize lane detection model with GPU, fallback to CPU if needed
        try:
            self.lane_detector = UltrafastLaneDetector(model_path, model_type, use_gpu)
        except RuntimeError as e:
            self.get_logger().warn("GPU not available, switching to CPU.")
            self.lane_detector = UltrafastLaneDetector(model_path, model_type, False)

    def image_cb(self, img):
        bridge = CvBridge()
        cap = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        width, height = cap.shape[1], cap.shape[0]
        mid_line_img = None
        detected = 'False'

        # padding_size = 330
        # 좌측에 흰색 이미지 추가
        # padding_image = np.zeros((height, padding_size, 3), dtype=np.uint8) * 255  # 흰색 이미지 생성
        # cap = np.concatenate((padding_image, cap), axis=1)
        # cap = np.concatenate((cap, padding_image), axis=1)
        l_padding = 310
        r_padding = 350
        # 좌측에 흰색 이미지 추가
        l_padding_image = np.zeros((height, l_padding, 3), dtype=np.uint8) * 255
        r_padding_image = np.zeros((height, r_padding, 3), dtype=np.uint8) * 255
        # frame 좌우로 확장
        cap = np.concatenate((l_padding_image, cap), axis=1)
        cap = np.concatenate((cap, r_padding_image), axis=1)

        # UltrafastLaneDetector로 차선 검출
        final_detected, line_img, lateral_error, list_for_rviz = self.lane_detector.detect_lanes(cap)

        if final_detected:
            detected = True
            print(lateral_error)
        else:
            pass

        # pub result
        image_message = bridge.cv2_to_imgmsg(line_img, encoding="mono8")
        image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(image_message)

        # pub lateral error
        self.reliability_of_lateral_error(lateral_error)
        self.lateral_error.data = lateral_error
        if lateral_error < 1 and self.reliability_var < 0.005:
            self.lateral_error_pub.publish(self.lateral_error)

        # pub lane maker
        self.pub_lane_maker(list_for_rviz)
        
        # ✓✓✓ 새로 추가: 실시간 화면에 결과 이미지 표시 (lateral error 오버레이)
        display_img1 = line_img.copy()
        cv2.putText(display_img1, f"Lateral Error: {lateral_error:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Lane Detection", display_img1)
        cv2.waitKey(1)

        # ✓✓✓ 새로 추가: 원본 카메라 피드에 lateral error 오버레이하여 실시간으로 표시
        display_img2 = cap.copy()  # 원본 화면 사용
        cv2.putText(display_img2, f"Lateral Error: {lateral_error:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Raw Camera Feed", display_img2)
        cv2.waitKey(1)

    def pub_lane_maker(self, list_for_rviz):
        line_marker = Marker()
        line_marker.header.frame_id = "car"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1
        line_marker.scale.y = 0.1
        line_marker.scale.z = 0.1
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0

        for x, y in list_for_rviz:
            point = Point()
            # 90도 회전한 좌표 계산
            new_x = y
            new_y = -x
            point.x = new_x
            point.y = new_y
            line_marker.points.append(point)

        self.lanent_lane_pub.publish(line_marker)

    def reliability_of_lateral_error(self, lateral_error):

        self.lateral_error_array.append(lateral_error)
        if len(self.lateral_error_array) > 20:
            del self.lateral_error_array[0]
        self.reliability_var = np.var(self.lateral_error_array)
        reliability_IQR = iqr(self.lateral_error_array)
        print(f'IQR: {reliability_IQR}')
        print(f'Variance: {self.reliability_var}')

        # outlier 검출
        outliers = []
        for value in self.lateral_error_array:
            if value < np.percentile(lateral_error, q=25) - (1.5 * reliability_IQR) or value > np.percentile(lateral_error, q=75) + (1.5 * reliability_IQR):
                outliers.append(value)
        print(f'이상치: {outliers}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneNet()
        rclpy.spin(node)
    except Exception as e:
        print("An error occurred:")
        traceback.print_exc()  # ✓ 새로 추가: node.traceback 대신 traceback 모듈 사용하여 예외 출력
    finally:
        if node:
            node.destroy_node()
            print("Node destroyed.")
        rclpy.shutdown()
        print("ROS shutdown.")

if __name__ == "__main__":
    main()
