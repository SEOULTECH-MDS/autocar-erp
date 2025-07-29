#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
import numpy as np
from numpy import random

from cv_bridge import CvBridge
from perception.yolov7.models.experimental import attempt_load
from perception.yolov7.utils.datasets import letterbox
from perception.yolov7.utils.general import check_img_size, non_max_suppression, scale_coords
from perception.yolov7.utils.plots import plot_one_box
from perception.yolov7.utils.torch_utils import select_device, time_synchronized

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

# 설정 값
package_share = get_package_share_directory('perception')
WEIGHTS = os.path.join(package_share, 'yolov7', 'weights', 'tf_best.pt')
# WEIGHTS = 'weights/tf_best.pt'
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.50
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False

QUEUE_SIZE = 13
CLASS_MAP = ['Green', 'Left', 'Red', 'Straightleft', 'Yellow']


class YOLO(Node):
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
        self.queue_list = [[0 for _ in range(QUEUE_SIZE)] for _ in range(5)]
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
        self.device = select_device(DEVICE)
        self.half = self.device.type != 'cpu'
        self.get_logger().info(f"device: {self.device}")

        self.model = attempt_load(WEIGHTS, map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.imgsz = check_img_size(IMG_SIZE, s=self.stride)
        if self.half:
            self.model.half()

        # 클래스 이름 및 색상 설정
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # GPU 사용 시 한 번 더 워밍업
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))

    def callback_img(self, img_raw_msg):
        with torch.no_grad():
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

            # [✓] Modified: OpenCV 창에 실시간 이미지 표시
            cv2.imshow("Traffic Light Detection", cap if not self.yolo_mode else img_res)
            cv2.waitKey(1)

            self.img_res_pub.publish(img_res_msg)

    def detect(self, img0):
        # 입력 이미지 전처리: letterbox 리사이즈
        img = letterbox(img0, self.imgsz, stride=self.stride)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR -> RGB, (H,W,C) -> (C,H,W)
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 -> float
        img /= 255.0  # 0-255 -> 0.0-1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # 추론
        _ = time_synchronized()
        pred = self.model(img, augment=AUGMENT)[0]

        # NMS 적용
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=CLASSES, agnostic=AGNOSTIC_NMS)

        det = pred[0]
        ids = []
        if det is not None and len(det):
            # 검출 결과 박스를 원본 이미지 크기로 변환
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

            for *xyxy, conf, cls in reversed(det):
                obj_id = int(cls)
                ids.append(obj_id)
                label = f'{self.names[obj_id]} {conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=self.colors[obj_id], line_thickness=3)
                # bounding box 좌표 추출 (필요시 추가 연산 가능)
                xmin, ymin, xmax, ymax = [int(t.item()) for t in xyxy]
                # 여기서는 단순히 검출이 된 경우 1을 투표
                self.queue_list[obj_id].append(1)

            # 해당 프레임에서 검출되지 않은 클래스엔 0 추가
            detected_ids = set(ids)
            for i in range(5):
                if i not in detected_ids:
                    self.queue_list[i].append(0)
        else:
            # 검출 결과가 없으면 모든 클래스에 0 추가
            for k in range(5):
                self.queue_list[k].append(0)

        return img0, list(set(ids))

    def hard_vote(self, queue):
        return sum(queue) > 0.7 * QUEUE_SIZE

    def callback_traffic_pub(self):
        final_check = String()
        data = ""
        # 각 클래스의 큐 크기를 QUEUE_SIZE로 유지
        for n in range(5):
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
    node = YOLO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        cv2.destroyAllWindows()  # [✓] Modified: OpenCV 창 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
