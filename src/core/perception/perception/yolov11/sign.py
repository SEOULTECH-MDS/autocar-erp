#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

# [✓] Modified: "models" alias 추가
import sys
import perception.yolov7.models as models
sys.modules['models'] = models
# [✓] Modified: "utils" alias 추가
import perception.yolov7.utils as utils
sys.modules['utils'] = utils

import time
import cv2
import torch
import statistics
import numpy as np
import sys
import argparse

from numpy import random
from cv_bridge import CvBridge
# YOLOv8 사용: ultralytics의 YOLO 모델을 import
from ultralytics import YOLO
from perception.yolov11.utils.plots import plot_one_box

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String, Int32
from geometry_msgs.msg import Pose, PoseArray

# 설정 값
package_share = get_package_share_directory('perception')
WEIGHTS = os.path.join(package_share, 'yolov11', 'weights', 'best_표지판.pt')
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.85
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False

QUEUE_SIZE = 13
# CLASS_MAP = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3']
CLASS_MAP = (
    ("id_0", "id_1", "id_2"),
    ("A1",),
    ("A2",),
    ("A3",),
    ("B1",),
    ("B2",),
    ("B3",)
)

class YOLOv11(Node):
    def __init__(self):
        super().__init__('sign_detector')

        self.detected_pub = self.create_publisher(Image, "/yolo/sign", 10)
        self.delivery_pub = self.create_publisher(Int32MultiArray, "/delivery_sign", 10)
        self.boungingboxes_pub = self.create_publisher(PoseArray, "/bounding_boxes/deliver", 10)
        self.target_sign_pub = self.create_publisher(Int32, "/target_sign", 10)
        
        self.create_subscription(String, "/driving_mode", self.callback_mode, 10)
        self.create_subscription(Image, "/camera_sign/image_raw", self.callback_img, 10)
        # self.create_subscription(Image, "/carla/ego_vehicle/rgb_right/image", self.callback_img, 10)

        self.img_width = IMG_SIZE
        # self.mode = None
        self.mode = 'delivery_start'
        self.sign = 0
        self.target_sign = None

        self.B1 = []
        self.B2 = []
        self.B3 = []

        self.queue_list = [[-1 for i in range(QUEUE_SIZE)] for j in range(len(CLASS_MAP))]
        self.id_to_queue_list = [self.queue_list[i] for i in range(len(CLASS_MAP)) for _ in range(len(CLASS_MAP[i]))]

        # Timer 생성 (단위는 초)
        self.create_timer(0.1, self.yolo_pub)

        # 디바이스 및 모델 초기화
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.half = self.device != 'cpu'
        self.get_logger().info(f"device: {self.device}")

        self.model = YOLO(WEIGHTS)
        self.model.to(self.device)
        if self.half:
            self.model.model.half()

        # 클래스 이름 및 색상 설정
        self.names = self.model.names
        if isinstance(self.names, dict):
            self.names = [self.names[i] for i in sorted(self.names.keys())]
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # GPU 사용 시 워밍업 (더미 추론)
        if self.device != 'cpu':
            dummy_img = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            _ = self.model(dummy_img, imgsz=IMG_SIZE, conf=CONF_THRES, iou=IOU_THRES, augment=AUGMENT)

    def callback_mode(self, msg):
        self.mode = msg.data

    def callback_img(self, img, event=None):
        self.img_width = img.width
        bridge = CvBridge()
        cap = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()  # 현재 시간
        poses.header.frame_id = 'yolo'
        
        if self.mode == 'delivery_start' or self.mode == 'delivery_finish':
            result, coords = self.detect(cap)
            image_message = bridge.cv2_to_imgmsg(result, encoding="bgr8")
            
            for coord in coords:
                pose = Pose()
                # 포즈의 위치 설정
                pose.position.x = float(coord[0])  # 0 blue or 1 yellow
                # pose.position.y = 0.  # confidence
                # pose.position.z = 0.
                    
                pose.orientation.x = float(coord[1])  # xmin
                pose.orientation.y = float(coord[2])  # ymin
                pose.orientation.z = float(coord[3])  # xmax
                pose.orientation.w = float(coord[4])  # ymax

                self.get_logger().info(
                    f"(xmin, ymin)=({pose.orientation.x:.0f}, {pose.orientation.y:.0f}) "
                    f"(xmax, ymax)=({pose.orientation.z:.0f}, {pose.orientation.w:.0f})"
                )

                poses.poses.append(pose)
            self.boungingboxes_pub.publish(poses)
            cv2.imshow("Real-time Result", result)
        else:
            image_message = bridge.cv2_to_imgmsg(cap, encoding="bgr8")
            cv2.imshow("Real-time Result", cap) 

        cv2.waitKey(1)

        # header stamp 갱신
        image_message.header.stamp = self.get_clock().now().to_msg()
        self.detected_pub.publish(image_message)

    def detect(self, frame):
        img0 = frame
        # YOLOv11 내부에서 리사이즈 및 전처리
        results = self.model(img0, conf=CONF_THRES, iou=IOU_THRES, augment=AUGMENT)
        coords = []
        
        result = results[0] # results는 리스트 형태
        boxes = result.boxes
        if len(boxes) > 0:
            # boxes.xyxy, boxes.conf, boxes.cls는 각각 텐서로 제공
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy()
            
            for i in range(len(xyxy)):
                detection = xyxy[i]
                conf = confs[i]
                cls = int(clss[i])
                label = f'{self.names[cls]} {conf:.2f}'
                
                plot_one_box(detection, img0, label=label, color=self.colors[cls], line_thickness=3)
                xmin, ymin, xmax, ymax = [int(x) for x in detection]
                xmean = (xmin + xmax) / 2
                coords.append([cls, xmin, ymin, xmax, ymax])

                if xmean > 50 and xmean < self.img_width - 50:
                    if cls in (0, 1, 2):
                        self.id_to_queue_list[cls + 3].append(int(xmean))
                        self.id_to_queue_list[0].append(cls)
                        if self.target_sign is None:
                            if cls == 0:
                                self.target_sign = 3
                            elif cls == 1:
                                self.target_sign = 4
                            elif cls == 2:
                                self.target_sign = 5
                        self.get_logger().info(f"Delivery start mode activated. Detected sign: {self.names[cls]}")
                        self.get_logger().info(f"Delivery start mode activated. Target sign: {self.names[self.target_sign]}")
                        if xmean >= 550:
                            self.get_logger().info(f"Start sign A{self.sign} detected Stop.")
                            self.start_detected = True
                    
                    msg = Int32()
                    if self.target_sign is not None:
                        msg.data = int(self.target_sign)
                    else:
                        msg.data = 0  # 또는 원하는 기본값
                    self.target_sign_pub.publish(msg)
                    
                    if cls == self.target_sign:
                        self.get_logger().info(f"Target sign B{self.sign} detected.")
                        self.target_detected = True
                        if xmean >= 550:
                            self.get_logger().info(f"Target sign B{self.sign} detected Stop.")
                    
        else:
            for queue in self.queue_list:
                if len(queue) == QUEUE_SIZE:
                    queue.append(-1)
                    
        if len(self.B1):
            bx = min(self.B1)
            self.id_to_queue_list[6].append(bx)
        if len(self.B2):
            bx = min(self.B2)
            self.id_to_queue_list[7].append(bx)
        if len(self.B3):
            bx = min(self.B3)
            self.id_to_queue_list[8].append(bx)

        self.B1 = []
        self.B2 = []
        self.B3 = []

        return img0, coords

    def delivery_vote(self, queue):
        if queue.count(-1) > int(QUEUE_SIZE / 2):
            return 0
        else:
            for element in queue:
                if element != -1:
                    val = element
            return val

    def hard_vote(self, queue):
        return statistics.mode(queue)

    def yolo_pub(self):
        final_check = Int32MultiArray()

        for queue in self.queue_list:
            while len(queue) != QUEUE_SIZE:
                del queue[0]

        queue_list = self.queue_list

        for idx in range(len(queue_list)):
            if idx == 0:
                final_check.data.append(self.hard_vote(queue_list[idx]))
            else:
                final_check.data.append(self.delivery_vote(queue_list[idx]))

        if final_check.data[0] != -1:
            self.sign = final_check.data[0] + 1

        if self.sign:
            pass
        else:
            self.get_logger().info("Not Detected yet")

        self.delivery_pub.publish(final_check)
    
def main(args=None):
    rclpy.init(args=args)
    yolo = YOLOv11()
    rclpy.spin(yolo)
    yolo.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()