#!/usr/bin/env python3
import numpy as np

from perception.tracker.src.motrackers.iou_tracker import IOUTracker
from perception.tracker.src.motrackers.utils.misc import draw_tracks

import rclpy
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image

import cv2


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('bounding_boxes_tracker')

        self.bridge = CvBridge()
        self.tracker = IOUTracker(max_lost=3, iou_threshold=0.4, min_detection_confidence=0.4, max_detection_confidence=0.7,
                                  tracker_output_format='visdrone_challenge')
        
        # self.img_sub = message_filters.Subscriber(self, Image, '/yolo/rubber')
        # self.bbox_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/rubber')
        self.bbox_sub = self.create_subscription(PoseArray, '/bounding_boxes/rubber', self.callback_tracker, 10)
        
        # self.img_tracked_pub = self.create_publisher(Image, "/image_tracked", 10)
        self.bbox_tracked_pub = self.create_publisher(PoseArray, '/bounding_boxes/tracked', 10)
        
        # self.sync = message_filters.ApproximateTimeSynchronizer([self.bbox_sub, self.img_sub], queue_size=100, slop=0.5, allow_headerless=True)
        # self.sync.registerCallback(self.callback_tracker)
    
    def callback_tracker(self, bbox_msg): # self, bbox_msg, img_msg
        # Image
        # img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        
        bboxes = []
        confidences = []
        labels = []
        for bbox in bbox_msg.poses:
            bboxes.append([int(bbox.orientation.x), int(bbox.orientation.y), int(bbox.orientation.z - bbox.orientation.x), int(bbox.orientation.w - bbox.orientation.y)])
            confidences.append(bbox.position.y)
            labels.append(bbox.position.x)
        
        tracks = self.tracker.update(bboxes, confidences, labels)
        # updated_image = draw_tracks(img, tracks)
        
        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = 'yolo'

        # for _, _, x_min, y_min, width, height, _, lbl, _, _ in tracks:
            
                
        # updated_img_msg = self.bridge.cv2_to_imgmsg(updated_image, encoding="bgr8")
        # updated_img_msg.header.stamp = self.get_clock().now().to_msg()
        # self.img_tracked_pub.publish(updated_img_msg)

        # cv2.imshow("YOLO Rubber Detection", updated_image)
        # cv2.waitKey(1)
        
        return
    

def main(args=None):
    rclpy.init(args=args)
    tracker = ObjectTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()