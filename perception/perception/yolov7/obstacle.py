#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
import time
import cv2

from numpy import random
from perception.yolov7.models.experimental import attempt_load
from perception.yolov7.utils.datasets import letterbox
from perception.yolov7.utils.general import check_img_size, check_requirements, non_max_suppression, scale_coords
from perception.yolov7.utils.plots import plot_one_box
from perception.yolov7.utils.torch_utils import select_device, time_synchronized

# [✓] Modified: "models" alias 추가
import sys
import perception.yolov7.models as models
sys.modules['models'] = models
# [✓] Modified: "utils" alias 추가
import perception.yolov7.utils as utils
sys.modules['utils'] = utils

# 하이퍼파라미터 및 설정
package_share = get_package_share_directory('perception')
WEIGHTS = os.path.join(package_share, 'yolov7', 'weights', 'rubber_new.pt')
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.60
IOU_THRES = 0.35
CLASSES = None
AGNOSTIC_NMS = False

class YOLO(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # 이미지 메시지를 구독할 서브스크라이버 생성
        self.subscription = self.create_subscription(Image, '/image_combined', self.image_callback, 10)
        # self.subscription = self.create_subscription(Image, '/camera_obstacle/image_raw', self.image_callback, 10)
        self.subscription  # 사용하지 않는 변수 경고 방지

        # PoseArray 메시지를 퍼블리시할 퍼블리셔 생성
        self.pose_array_pub = self.create_publisher(PoseArray, '/bounding_boxes/obstacle', 10)
        self.img_res_pub = self.create_publisher(Image, '/yolo/obstacle', 10)
        # self.obstacle_pub = self.create_publisher(String, '/obstacle_type', 10)
        
        # CvBridge 초기화 (ROS 이미지와 OpenCV 이미지 간 변환)
        self.bridge = CvBridge()

        # YOLO 모델 초기화: 장치 선택, 모델 로딩, 이미지 사이즈 등 설정
        self.device = select_device(DEVICE)
        self.half = self.device.type != 'cpu'  # GPU 사용 시 half precision 적용
        self.get_logger().info(f"Using device: {self.device}")

        # 모델 로딩
        self.model = attempt_load(WEIGHTS, map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.imgsz = check_img_size(IMG_SIZE, s=self.stride)
        if self.half:
            self.model.half()  # FP16 변환

        # 모델이 검출할 클래스 이름과 각 클래스별 랜덤 색상 설정
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # GPU를 사용할 경우 모델 워밍업
        if self.device.type != 'cpu':
            dummy_input = torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters()))
            self.model(dummy_input)
        
        # 필수 패키지 요구사항 확인
        check_requirements(exclude=('pycocotools', 'thop'))

        self.get_logger().info("YOLO Detector node has been started.")

    def image_callback(self, image_msg):
        with torch.no_grad():
            start_time = time.perf_counter()
            
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"CV Bridge error: {e}")
                return
            
            # 객체 검출 수행
            rubbers = self.detect(cv_image)
            if rubbers is None:
                # [✓] Modified: 검출 결과가 없을 때도 원본 이미지를 표시
                cv2.imshow("YOLO Obstacle Detection", cv_image)
                cv2.waitKey(1)
                return
            
            # [✓] Modified: 검출 결과를 바탕으로 원본 이미지에 바운딩 박스 그리기
            for det in rubbers:
                cls, xmin, ymin, xmax, ymax, conf = det
                label = f'{self.names[cls]} {conf:.2f}'
                plot_one_box([xmin, ymin, xmax, ymax], cv_image, label=label, color=self.colors[cls], line_thickness=2)
            
            # [✓] Modified: 결과 이미지를 실시간으로 디스플레이
            cv2.imshow("YOLO Obstacle Detection", cv_image)
            cv2.waitKey(1)
            
            # 결과 이미지를 ROS 이미지 메시지로 변환 후 퍼블리시
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_message.header.stamp = self.get_clock().now().to_msg()
            self.img_res_pub.publish(image_message)
            
            # 검출 결과를 PoseArray 메시지로 구성
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'yolo'
            
            for rubber in rubbers:
                pose = Pose()
                # 각 필드에 검출 결과 할당 (클래스, 바운딩 박스 좌표, 신뢰도)
                # [✓] Modified: 각 필드에 float 형식으로 검출 결과 할당 (ROS msg 타입은 float이어야 함)
                pose.position.x = float(rubber[0])  # 클래스 인덱스
                pose.position.y = float(rubber[5])  # 신뢰도
                # pose.position.z 는 0으로 유지됨
                pose.orientation.x = float(rubber[1])  # xmin
                pose.orientation.y = float(rubber[2])  # ymin
                pose.orientation.z = float(rubber[3])  # xmax
                pose.orientation.w = float(rubber[4])  # ymax
                
                pose_array.poses.append(pose)
            
            # 검출 결과 퍼블리시
            self.pose_array_pub.publish(pose_array)
            
            elapsed_time = time.perf_counter() - start_time
            self.get_logger().info(f"YOLO detection time: {elapsed_time:.5f} seconds")
    
    def detect(self, img0):
        # 이미지 전처리: letterbox로 크기 조정 및 패딩 추가
        img = letterbox(img0, self.imgsz, stride=self.stride)[0]
        # BGR → RGB 변환, 차원 순서 변경 및 메모리 연속성 확보
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)

        # 이미지를 torch 텐서로 변환
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # 데이터 타입 변환
        img /= 255.0  # 0~255 범위를 0~1 범위로 정규화
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        # 추론 수행
        pred = self.model(img, augment=AUGMENT)[0]
        
        # Non-Maximum Suppression 적용
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=CLASSES, agnostic=AGNOSTIC_NMS)
        
        det = pred[0]
        
        if det is not None and len(det):
            # 검출된 바운딩 박스 좌표를 원본 이미지 크기에 맞게 스케일 조정
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            rubbers = []
            for *xyxy, conf, cls in reversed(det):
                xmin, ymin, xmax, ymax = [int(tensor.item()) for tensor in xyxy]
                rubber = [int(cls), xmin, ymin, xmax, ymax, conf]
                rubbers.append(rubber)
            return rubbers
        return None
    '''
    def callback_obstacle_pub(self):
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

        self.get_logger().info(f"Obstacle type: {data}")
        final_check.data = data
        self.obstacle_pub.publish(final_check)
    '''
def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YOLO()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
