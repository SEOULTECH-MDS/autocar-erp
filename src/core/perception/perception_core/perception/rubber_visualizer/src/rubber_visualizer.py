#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
from perception.rubber_visualizer.src.visualizer_handler import *

class RubberVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('rubber_visualizer')

        self.bridge = CvBridge()

        self.image_sub_side = message_filters.Subscriber(self, Image, '/camera_side/image_raw')
        self.image_sub_front = message_filters.Subscriber(self, Image, '/camera_front/image_raw')
        # self.image_sub = message_filters.Subscriber(self, Image, '/usb_cam_1/image_raw')
        self.cluster_2d_sub = message_filters.Subscriber(self, PoseArray, '/clusters_2d')
        self.bbox_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/rubber')
        self.bbox_tracked_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/tracked')

        # self.img_result_pub = self.create_publisher(Image, '/image_rubber_result', 10)
        self.img_result_side_pub = self.create_publisher(Image, '/image_rubber_result_side', 10)
        self.img_result_front_pub = self.create_publisher(Image, '/image_rubber_result_front', 10)

        sub_list = [self.image_sub_side, self.image_sub_front, self.cluster_2d_sub, self.bbox_sub, self.bbox_tracked_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(sub_list, queue_size=10, slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_perception)

        # self.get_logger().info('RubberVisualizer node started.')

    # def callback_perception(self, img_msg, clusters_2d_msg, bboxes_msg, bboxes_tracked_msg):

    #     # 1) ROS → OpenCV
    #     img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    #     # 2) 파싱
    #     clusters_2d = get_cluster_2d(clusters_2d_msg)
    #     bboxes, bbox_labels = get_bbox(bboxes_msg)
    #     bboxes_tracked, bbox_tracked_labels = get_bbox(bboxes_tracked_msg)

    #     # 3) 시각화 (in‑place)
    #     visualize_cluster_2d(clusters_2d, img)
    #     visualize_bbox(bboxes, bbox_labels, img)
    #     visualize_bbox_tracked(bboxes_tracked, bbox_tracked_labels, img)

    #     # 4) OpenCV → ROS Image 변환 후 발행
    #     img_out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
    #     img_out.header.stamp = self.get_clock().now().to_msg()
    #     self.img_result_pub.publish(img_out)

    def callback_perception(self, img_side_msg, img_front_msg, clusters_2d_msg, bboxes_msg, bboxes_tracked_msg):
        """
        두 카메라 이미지와 검출 결과들을 동시에 처리하는 콜백 함수
        """

        # 1) ROS → OpenCV 변환 (두 이미지 모두)
        img_side = self.bridge.imgmsg_to_cv2(img_side_msg, desired_encoding='bgr8')
        img_front = self.bridge.imgmsg_to_cv2(img_front_msg, desired_encoding='bgr8')

        # 2) 검출 결과 파싱
        clusters_2d = get_cluster_2d(clusters_2d_msg)
        bboxes, bbox_labels = get_bbox(bboxes_msg)
        bboxes_tracked, bbox_tracked_labels = get_bbox(bboxes_tracked_msg)

        # 3) 사이드 카메라 이미지 시각화 (in-place)
        img_side_processed = img_side.copy()  # 원본 보존을 위한 복사
        visualize_cluster_2d(clusters_2d, img_side_processed)
        visualize_bbox(bboxes, bbox_labels, img_side_processed)
        visualize_bbox_tracked(bboxes_tracked, bbox_tracked_labels, img_side_processed)

        # 4) 프론트 카메라 이미지 시각화 (in-place)
        img_front_processed = img_front.copy()  # 원본 보존을 위한 복사
        visualize_cluster_2d(clusters_2d, img_front_processed)
        visualize_bbox(bboxes, bbox_labels, img_front_processed)
        visualize_bbox_tracked(bboxes_tracked, bbox_tracked_labels, img_front_processed)

        # 5) OpenCV → ROS Image 변환 후 발행
        # 사이드 카메라 결과 발행
        img_side_out = self.bridge.cv2_to_imgmsg(img_side_processed, encoding='bgr8')
        img_side_out.header.stamp = self.get_clock().now().to_msg()
        img_side_out.header.frame_id = "camera_side"
        self.img_result_side_pub.publish(img_side_out)

        # 프론트 카메라 결과 발행
        img_front_out = self.bridge.cv2_to_imgmsg(img_front_processed, encoding='bgr8')
        img_front_out.header.stamp = self.get_clock().now().to_msg()
        img_front_out.header.frame_id = "camera_front"
        self.img_result_front_pub.publish(img_front_out)

def main() -> None:
    rclpy.init()
    visualizer = RubberVisualizer()
    rclpy.spin(visualizer)

    # 노드 종료 처리
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()