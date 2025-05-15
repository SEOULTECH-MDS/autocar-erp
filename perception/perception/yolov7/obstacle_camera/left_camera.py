#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraExample(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.bridge = CvBridge()

        # Set the desired resolution
        width, height = 640, 360

        # Set the camera's resolution
        self.cap = cv2.VideoCapture(0) # 나중에 카메라 번호에 맞게 수정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video file')
            return

        # Publish video at 30Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.cam_exam = self.create_publisher(Image, '/image_left', 10)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        mtx = np.array([[474.2048 , 0  , 319.6910 ], 
                        [0  , 476.7333 , 204.9360    ], 
                        [  0.        ,   0.        ,   1.        ]])
        dist = np.array([[-0.3557 , 0.1115, 0. , 0.0 , 0.0 ]])  
        # mtx = np.array([[630.304667 , 0  , 318.199702 ], 
        #                 [0  , 631.354012 ,  262.739835   ], 
        #                 [  0.        ,   0.        ,   1.        ]])
        # dist = np.array([[-0.360728 , 0.114289 , -0.001610 , -0.000133 , 0.0 ]])

        # new_camera_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 360), 0, (640,360))

        
        new_camera_mtx = np.array([[381.18310547,   0.,         319.02992935],
                            [0. ,        448.0055542, 207.31755552],
                            [0.000000, 0.000000, 1.000000]])
        # left_new_cameramtx [[588.11367456   0.         325.73825108]
        #     [  0.         635.91648693 223.44224959]
        #     [  0.           0.           1.        ]]


        # print("left_new_cameramtx", new_camera_mtx)
        
        dst = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)

        img_msg = self.bridge.cv2_to_imgmsg(dst, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        # img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.cam_exam.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# cx = 405.22805
# cy = 318.98843
# fx = 789.71231
# fy = 791.33893
# distortion_model = plumb_bob
# dist = [-3.692636e-01 1.282285e-01 7.064332e-04 2.316315e-04 -1.097098e-02 0.000000e+00