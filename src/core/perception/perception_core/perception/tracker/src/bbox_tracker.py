#!/usr/bin/env python3
import numpy as np

from perception.tracker.src.motrackers.iou_tracker import IOUTracker

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

import os


class BboxTracker(Node):
    def __init__(self):
        super().__init__('bounding_boxes_tracker')

        # IOU 기반 트래커
        self.tracker = IOUTracker(
            max_lost=5,
            iou_threshold=0.4,
            min_detection_confidence=0.3,
            max_detection_confidence=1.0,
            tracker_output_format='visdrone_challenge'
        )

        # Subscriber
        self.bbox_sub = self.create_subscription(PoseArray, '/bounding_boxes/rubber', self.callback_tracker, 10)

        # Publisher
        self.bbox_tracked_pub = self.create_publisher(PoseArray, '/bounding_boxes/tracked', 10)

        # 콘솔 초기화 및 안내 메시지
        os.system('clear')
        self.get_logger().info("\033[1;33m Tracking YoLo bounding boxes. \033[0m")

    def callback_tracker(self, bbox_msg):
        bboxes = []
        confidences = []
        labels = []
        for bbox in bbox_msg.poses:
            bboxes.append([float(bbox.orientation.x), float(bbox.orientation.y),
                           float(bbox.orientation.z - bbox.orientation.x),
                           float(bbox.orientation.w - bbox.orientation.y)])
            # bboxes.append([bbox.orientation.x, bbox.orientation.y, bbox.orientation.z-bbox.orientation.x, bbox.orientation.w-bbox.orientation.y])
            confidences.append(bbox.position.y)
            labels.append(bbox.position.x)

        tracks = self.tracker.update(bboxes, confidences, labels)

        bounding_boxes = PoseArray()
        bounding_boxes.header.stamp = self.get_clock().now().to_msg()
        bounding_boxes.header.frame_id = 'yolo'
        for _, _, x_min, y_min, width, height, _, lbl, _, _ in tracks:
            bbox = Pose()
            bbox.position.x = float(lbl)  # 0 blue or 1 yellow

            bbox.orientation.x = float(x_min)          # xmin
            bbox.orientation.y = float(y_min)          # ymin
            bbox.orientation.z = float(x_min + width)  # xmax
            bbox.orientation.w = float(y_min + height) # ymax

            bounding_boxes.poses.append(bbox)

        self.bbox_tracked_pub.publish(bounding_boxes)


def main(args=None):
    rclpy.init(args=args)
    tracker_node = BboxTracker()
    try:
        rclpy.spin(tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        tracker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
