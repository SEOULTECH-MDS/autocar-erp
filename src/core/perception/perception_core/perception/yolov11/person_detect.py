#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory

import time
import cv2
import torch
import numpy as np
from numpy import random

from ultralytics import YOLO
from cv_bridge import CvBridge
from perception.yolov11.utils.plots import plot_one_box  # 시각화 유틸
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray

package_share = get_package_share_directory('perception')
WEIGHTS = os.path.join(package_share, 'yolov11', 'weights', 'yolo11l.pt')
IMG_SIZE = 640
CONF_THRES = 0.60
IOU_THRES = 0.45
AUGMENT = False

PERSON_CLASS_ID = 0
FRAME_ID = 'yolo'  # PoseArray header.frame_id


class YoloPersonNode(Node):
    def __init__(self):
        super().__init__('obstacle_person')

        # ROS I/O
        self.img_sub = self.create_subscription(Image, '/camera_obstacle/image_raw', self.callback_img, 10)
        # self.img_sub = self.create_subscription(Image, '/usb_cam_1/image_raw', self.callback_img, 10)
        self.pose_pub = self.create_publisher(PoseArray, '/bounding_boxes/person', 10)
        self.img_pub = self.create_publisher(Image, '/image_result/person', 10)

        self.bridge = CvBridge()

        # DISPLAY 유무 확인(헤드리스면 imshow 생략)
        self.display_ok = self._check_display_support()
        if not self.display_ok:
            self.get_logger().warn("GUI 지원이 없어 화면 표시(cv2.imshow)를 생략합니다.")

        # 디바이스/half 설정
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.half = self.device != 'cpu'
        # self.get_logger().info(f"Using device: {self.device}")

        # 모델 로드
        self.model = YOLO(WEIGHTS)
        self.model.to(self.device)
        if self.half:
            self.model.model.half()

        # 클래스 이름 설정
        self.names = self.model.names
        if isinstance(self.names, dict):
            # dict 형태면 id 오름차순으로 정렬
            self.names = [self.names[i] for i in sorted(self.names.keys())]

        # 색상 팔레트(클래스별 고정 색) — 여기선 car 한 색상만 필요하지만 통일
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]
        # if CAR_CLASS_ID >= len(self.names):
        #     self.get_logger().warn(f"CAR_CLASS_ID {CAR_CLASS_ID}가 model.names 길이({len(self.names)})를 벗어납니다.")

        # GPU 워밍업
        if self.device != 'cpu':
            dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            _ = self.model(dummy, imgsz=IMG_SIZE, conf=CONF_THRES, iou=IOU_THRES, augment=AUGMENT, classes=[CAR_CLASS_ID])

        # self.get_logger().info("YOLO Car Detector node has been started.")

    def _check_display_support(self):
        """GUI 디스플레이 지원 여부를 안전하게 확인"""
        # DISPLAY 환경변수가 없으면 GUI 불가
        if not os.environ.get('DISPLAY'):
            return False
        
        # 실제 cv2.imshow가 작동하는지 테스트
        try:
            # 더미 이미지로 테스트
            dummy_img = np.zeros((100, 100, 3), dtype=np.uint8)
            test_window = "opencv_test_window"
            cv2.imshow(test_window, dummy_img)
            cv2.waitKey(1)
            cv2.destroyWindow(test_window)
            return True
        except cv2.error:
            return False
        except Exception:
            return False

    def callback_img(self, img_msg: Image):
        t0 = time.perf_counter()

        # ROS Image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            # self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 추론 (Ultralytics가 리사이즈/전처리/NMS 내부 처리)
        results = self.model(
            frame,
            imgsz=IMG_SIZE,
            conf=CONF_THRES,
            iou=IOU_THRES,
            augment=AUGMENT,
            classes=[PERSON_CLASS_ID]  # 사람만
        )
        result = results[0]

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = FRAME_ID

        # 박스가 없으면 화면만 업데이트하고 종료
        if len(result.boxes) == 0:
            if self.display_ok:
                try:
                    cv2.imshow("YOLOv11 Car Detection", frame)
                    cv2.waitKey(1)
                except cv2.error:
                    self.display_ok = False  # GUI 지원 불가능하면 비활성화
            # 빈 PoseArray도 퍼블리시(구독자 동기화용)
            self.pose_pub.publish(pose_array)
            return

        # 박스 그리기 & PoseArray 구성 & 로그 출력
        for box in result.boxes:
            # 클래스/신뢰도
            cls_id = int(box.cls.item())
            if cls_id != PERSON_CLASS_ID:
                # classes=[0]로 이미 필터되지만 안전망
                continue
            conf = float(box.conf.item())
            cls_name = self.names[cls_id] if cls_id < len(self.names) else f"id_{cls_id}"

            # 좌표(xyxy)
            xyxy = box.xyxy.detach().view(-1).cpu().tolist()
            xmin, ymin, xmax, ymax = [int(v) for v in xyxy]

            # 로그: 클래스 이름/신뢰도/좌표
            # self.get_logger().info(
            #     f"[car] conf={conf:.2f}  bbox(xmin,ymin,xmax,ymax)=({xmin},{ymin},{xmax},{ymax})  name={cls_name}"
            # )

            # 시각화
            label = f"{cls_name} {conf:.2f}"
            color = self.colors[cls_id] if cls_id < len(self.colors) else (0, 255, 0)
            plot_one_box([xmin, ymin, xmax, ymax], frame, label=label, color=color, line_thickness=3)

            # PoseArray(규칙: pos.x=class, pos.y=conf, ori.xyzw=xmin/ymin/xmax/ymax)
            pose = Pose()
            pose.position.x = float(cls_id)
            pose.position.y = float(conf)
            pose.orientation.x = float(xmin)
            pose.orientation.y = float(ymin)
            pose.orientation.z = float(xmax)
            pose.orientation.w = float(ymax)
            pose_array.poses.append(pose)

        # 퍼블리시
        self.pose_pub.publish(pose_array)

        # 실시간 화면 표시
        if self.display_ok:
            try:
                cv2.imshow("YOLOv11 Car Detection", frame)
                cv2.waitKey(1)
            except cv2.error:
                self.display_ok = False  # GUI 지원 불가능하면 비활성화
        
        # 결과 이미지 퍼블리시
        try:
            img_res_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_res_msg.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(img_res_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")
        
        t_elapsed = time.perf_counter() - t0
        # self.get_logger().info(f"Inference time: {t_elapsed:.4f}s  (cars: {len(pose_array.poses)})")


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        if node.display_ok:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass  # GUI 지원이 없으면 무시
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()