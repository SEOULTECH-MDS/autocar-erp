#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

import os
CAMERA_HEIGHT = 360
CAMERA_WIDTH = 1280

class CombineCamera(Node):
    def __init__(self):
        super().__init__('combine_camera')
        self.bridge = CvBridge()
        
        # ROS
        # (ROS 노드 초기화는 main()에서 rclpy.init()으로 수행합니다.)
        self.left_sub = message_filters.Subscriber(self, Image, "/image_left")
        self.right_sub = message_filters.Subscriber(self, Image, "/image_right")
        # self.left_sub = message_filters.Subscriber(self, Image, "/left_image")
        # self.right_sub = message_filters.Subscriber(self, Image, "/right_image")
        # self.left_sub = message_filters.Subscriber(self, Image, "/camera_left/camera1/usb_cam/image_raw")
        # self.right_sub = message_filters.Subscriber(self, Image, "/camera_right/camera2/usb_cam/image_raw")
        
        self.sync = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub],
                                                                 queue_size=5,
                                                                 slop=0.5,
                                                                 allow_headerless=True)
        self.sync.registerCallback(self.callback_combine)
        
        self.combined_pub = self.create_publisher(Image, "/image_combined", 10)
        
        os.system('clear')
        print("\033[1;33m Combining left and right images. \033[0m")
        return
    
    def callback_combine(self, left_msg, right_msg):
        left_img = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_img = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        
        combined_img = np.hstack([left_img, right_img])
        image_message = self.bridge.cv2_to_imgmsg(combined_img, encoding="bgr8")
        image_message.header.stamp = self.get_clock().now().to_msg()
        self.combined_pub.publish(image_message)
        
        return

def main(args=None):
    rclpy.init(args=args)
    combine_cam = CombineCamera()
    try:
        rclpy.spin(combine_cam)
    except KeyboardInterrupt:
        pass
    finally:
        combine_cam.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()