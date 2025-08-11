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

        self.image_sub = message_filters.Subscriber(self, Image, '/image_combined')
        self.cluster_2d_sub = message_filters.Subscriber(self, PoseArray, '/clusters_2d')
        self.bbox_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/rubber')
        self.bbox_tracked_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/tracked')

        self.img_result_pub = self.create_publisher(Image, '/image_rubber_result', 10)

        sub_list = [self.image_sub, self.cluster_2d_sub, self.bbox_sub, self.bbox_tracked_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(sub_list, queue_size=10, slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_perception)

        # self.get_logger().info('RubberVisualizer node started.')

    def callback_perception(self, img_msg, clusters_2d_msg, bboxes_msg, bboxes_tracked_msg):

        # 1) ROS → OpenCV
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # 2) 파싱
        clusters_2d = get_cluster_2d(clusters_2d_msg)
        bboxes, bbox_labels = get_bbox(bboxes_msg)
        bboxes_tracked, bbox_tracked_labels = get_bbox(bboxes_tracked_msg)

        # 3) 시각화 (in‑place)
        visualize_cluster_2d(clusters_2d, img)
        visualize_bbox(bboxes, bbox_labels, img)
        visualize_bbox_tracked(bboxes_tracked, bbox_tracked_labels, img)

        # 4) OpenCV → ROS Image 변환 후 발행
        img_out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_out.header.stamp = self.get_clock().now().to_msg()
        self.img_result_pub.publish(img_out)

        cv2.imshow("YOLO Rubber Detection", img)
        cv2.waitKey(1)

def main() -> None:
    rclpy.init()
    visualizer = RubberVisualizer()
    rclpy.spin(visualizer)

    # 노드 종료 처리
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()