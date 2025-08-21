#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraRight(Node):
    def __init__(self):
        super().__init__('video_right')
        self.bridge = CvBridge()

        # Set the desired resolution
        width, height = 640, 360

        # Set the camera's resolution
        self.cap = cv2.VideoCapture(6)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video file')
            return

        # Publish video at 30Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.cam_exam = self.create_publisher(Image, '/image_right', 10)
        # self.cam_exam_ = self.create_publisher(Image, '/raw_image_left', 10)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        mtx = np.array([[472.1570 , 0  , 325.9352 ], 
                        [0  , 473.3999 , 156.8928    ], 
                        [  0.        ,   0.        ,   1.        ]])
        dist = np.array([[-0.3703 , 0.1326, 0. , 0.0 , 0.0 ]])  
        # mtx = np.array([[635.762102 , 0  , 310.528472 ], 
        #                 [0  , 635.872939 ,  205.602578   ], 
        #                 [  0.        ,   0.        ,   1.        ]])
        # dist = np.array([[-0.38 , 0.11 , 0.001682 , 0.003889 , 0.0 ]])  

        # new_camera_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 360), 0, (640,360))
        
        new_camera_mtx = np.array([[378.68261719,   0.,         328.19930137],
                            [0. ,        443.68624878, 153.57524293],
                            [0.000000, 0.000000, 1.000000]])
        
        # print("new_camera_mtx : ",new_camera_mtx)
        # print("right_new_cameramtx", new_camera_mtx)
        
        dst = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)
        # x,y,w,h = roi
        # dst = dst[y:y+h, x:x+w]

        img_msg = self.bridge.cv2_to_imgmsg(dst, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        # img_msg_ = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.cam_exam.publish(img_msg)
        # self.cam_exam_.publish(img_msg_)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# cx = 414.25409
# cy = 274.82945
# fx = 790.50194
# fy = 791.41525
# distortion_model = plumb_bob
# dist = [-3.970213e-01 2.430401e-01 -3.236830e-03 -2.782626e-03 -1.699801e-01 0.000000e+00 0.000000e+00 0.000000e+00 ]