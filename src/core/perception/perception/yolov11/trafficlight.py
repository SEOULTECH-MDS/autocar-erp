#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
import sys

# ultralytics 패키지의 YOLO 클래스를 이용
from ultralytics import YOLO

import time
import cv2
import torch
import numpy as np
from numpy import random

from cv_bridge import CvBridge
from perception.yolov11.utils.plots import plot_one_box

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

# 설정 값
package_share = get_package_share_directory('perception')
WEIGHTS = os.path.join(package_share, 'yolov11', 'weights', 'best_신호등.pt')
IMG_SIZE = 640
DEVICE = '' 
AUGMENT = False
CONF_THRES = 0.50
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False

QUEUE_SIZE = 13
CLASS_MAP = ['Green', 'Left', 'Red', 'Straightleft']


class YOLOv11(Node):
    def __init__(self):
        super().__init__('traffic_detector')
        self.get_logger().info("Initializing traffic light detection node")
        
        self.get_logger().info(f"WEIGHTS path: {WEIGHTS}")
        if os.path.isfile(WEIGHTS):
            self.get_logger().info("Weights file found!")
        else:
            self.get_logger().error("Weights file NOT found!")
        
        # 변수 초기화
        self.yolo_mode = True
        self.queue_list = [[0 for _ in range(QUEUE_SIZE)] for _ in range(len(CLASS_MAP))]
        self.bridge = CvBridge()

        # ROS2 Subscriber / Publisher 생성
        self.img_sub = self.create_subscription(
            Image,
            '/camera_traffic/image_raw',
            self.callback_img,
            10)
        
        self.traffic_mode_sub = self.create_subscription(
            Bool,
            '/mode/traffic',
            self.callback_traffic_mode,
            10)
        
        self.img_res_pub = self.create_publisher(
            Image,
            '/yolo/traffic_light',
            10)
        
        self.traffic_pub = self.create_publisher(
            String,
            '/traffic_sign',
            10)
        
        self.timer = self.create_timer(0.1, self.callback_traffic_pub)

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

    def callback_img(self, img_raw_msg):
        try:
            # ROS2 이미지 메시지를 OpenCV 이미지로 변환
            cap = self.bridge.imgmsg_to_cv2(img_raw_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if self.yolo_mode:
            img_res, ids = self.detect(cap)
            img_res_msg = self.bridge.cv2_to_imgmsg(img_res, encoding="bgr8")
        else:
            img_res_msg = self.bridge.cv2_to_imgmsg(cap, encoding="bgr8")

        # OpenCV 창에 실시간 이미지 표시
        cv2.imshow("Traffic Light Detection", cap if not self.yolo_mode else img_res)
        cv2.waitKey(1)

        self.img_res_pub.publish(img_res_msg)

    def detect(self, img0):
        # 추론 (내부에서 자동으로 리사이징, 전처리 및 NMS 수행)
        results = self.model(img0, imgsz=IMG_SIZE, conf=CONF_THRES, iou=IOU_THRES, augment=AUGMENT)
        result = results[0]
        ids = []
        if len(result.boxes) > 0:
            for box in result.boxes:
                # box.xyxy: tensor shape (1,4) → flatten하여 (4,) 형태로 변환
                xyxy = box.xyxy.cpu().numpy().flatten()
                conf = float(box.conf.cpu().numpy())
                obj_id = int(box.cls.cpu().numpy())
                ids.append(obj_id)
                self.get_logger().info(f"검출된 클래스 인덱스: {obj_id}, 신뢰도: {conf:.2f}")

                if obj_id >= len(self.names):
                    self.get_logger().error(f"클래스 인덱스 {obj_id}가 model names 범위를 초과합니다: {self.names}")
                    label = f"Unknown {conf:.2f}"
                else:
                    label = f'{self.names[obj_id]} {conf:.2f}'

                plot_one_box(xyxy, img0, label=label, color=self.colors[obj_id], line_thickness=3)
                # bounding box 좌표 추출 (필요시 추가 연산 가능)
                xmin, ymin, xmax, ymax = [int(t.item()) for t in xyxy]
                # 여기서는 단순히 검출이 된 경우 1을 투표
                self.queue_list[obj_id].append(1)
            
            # 해당 프레임에서 검출되지 않은 클래스에는 0 추가
            detected_ids = set(ids)
            for i in range(len(self.queue_list)):
                if i not in detected_ids:
                    self.queue_list[i].append(0)
        else:
            # 검출 결과가 없으면 모든 클래스에 0 추가
            for k in range(len(self.queue_list)):
                self.queue_list[k].append(0)

        return img0, list(set(ids))

    def hard_vote(self, queue):
        return sum(queue) > 0.7 * QUEUE_SIZE

    def callback_traffic_pub(self):
        final_check = String()
        data = ""
        # 각 클래스의 큐 크기를 QUEUE_SIZE로 유지하면서 투표 결과 확인
        for n in range(len(self.queue_list)):
            while len(self.queue_list[n]) > QUEUE_SIZE:
                del self.queue_list[n][0]
            if self.hard_vote(self.queue_list[n]):
                data += CLASS_MAP[n] if data == "" else f",{CLASS_MAP[n]}"

        if data == "":
            data = "None"

        self.get_logger().info(f"Traffic Light: {data}")
        final_check.data = data
        self.traffic_pub.publish(final_check)

    def callback_traffic_mode(self, tm_msg):
        self.yolo_mode = tm_msg.data


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv11()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        cv2.destroyAllWindows()  # OpenCV 창 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
