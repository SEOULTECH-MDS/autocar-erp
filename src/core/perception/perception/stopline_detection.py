#!/usr/bin/env python3
import cv2
import numpy as np
import time
import os  # [✔ ADDED] 동영상 경로를 구성하기 위해 import

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class StopLineDetect(Node):
    def __init__(self):
        super().__init__('stopline_detect')
        self.bridge = CvBridge() 
        
        # 차량-카메라 좌표 변환 행렬 구성
        R_veh2cam = np.transpose(rotation_from_euler(0., np.radians(7.0), 0.))
        T_veh2cam = translation_matrix((2., 0.0, -1.0))

        # 카메라 좌표계에 맞추기 위한 추가 회전
        R = np.array([[0., -1., 0., 0.],
                      [0., 0., -1., 0.],
                      [1., 0., 0., 0.],
                      [0., 0., 0., 1.]])
        self.RT = R @ R_veh2cam @ T_veh2cam
        
        # Subscriber: /image_lane 토픽에서 이미지 수신
        self.img_raw_sub = self.create_subscription(
            Image, '/image_lane', self.callback_img_raw, 10)
        
        # Publisher: 처리된 정지선 이미지와 오차 값 퍼블리시
        self.img_stop_line_pub = self.create_publisher(Image, '/stopline_img', 3)
        self.ate_pub = self.create_publisher(Float32, '/at_error_local', 10)
        # self.get_logger().info("StopLineDetect 노드가 초기화되었습니다.")
        
        # [✔ ADDED] 동영상 저장 관련 설정
        home_dir = os.path.expanduser('~')  # 사용자 홈 디렉토리 경로
        self.video_path = os.path.join(home_dir, 'Videos', 'stopline_detect.mp4')
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # mp4 코덱
        self.out = None  # VideoWriter는 첫 프레임 수신 시점에 초기화 예정

    def callback_img_raw(self, img_raw_msg):
        start_time = time.time()  
        
        # ROS2 메시지를 OpenCV 이미지로 변환
        img_raw_stop = self.bridge.imgmsg_to_cv2(img_raw_msg, desired_encoding='bgr8')
        """
        try:
            img_raw_stop = self.bridge.imgmsg_to_cv2(img_raw_msg, desired_encoding='bgr8')
            self.get_logger().debug(f"수신 이미지 shape: {img_raw_stop.shape}")
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 오류: {e}")
            return
        """

        # [✔ ADDED] 실시간 카메라 화면 모니터링 (웹캠 연결시)
        cv2.imshow("Raw Camera Feed", img_raw_stop)
        cv2.waitKey(1)

        # [✔ ADDED] VideoWriter 초기화 (첫 프레임 수신 시점)
        if self.out is None:
            height, width, channels = img_raw_stop.shape
            self.out = cv2.VideoWriter(self.video_path, self.fourcc, 30.0, (width, height))
        
        # [✔ ADDED] 원본 프레임(또는 필요에 따라 가공된 프레임)을 파일로 저장
        self.out.write(img_raw_stop)
        
        # 정지선 검출 수행
        stop_img = self.stopline(img_raw_stop)

        # [✔ ADDED] 검출 결과 실시간 모니터링 창에 표시
        scale_factor = 2  # 원하는 배율 (예: 2배)
        enlarged_img = cv2.resize(stop_img, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Stop Line Detection Result", enlarged_img)
        # cv2.imshow("Stop Line Detection Result", stop_img)
        cv2.waitKey(1)
        
        # 결과 이미지를 ROS2 메시지로 변환 후 타임스탬프 설정
        img_msg_stop = self.bridge.cv2_to_imgmsg(stop_img, encoding="bgr8")
        img_msg_stop.header.stamp = self.get_clock().now().to_msg()
        self.img_stop_line_pub.publish(img_msg_stop)
        """
        try:
            img_msg_stop = self.bridge.cv2_to_imgmsg(stop_img, encoding="bgr8")
            img_msg_stop.header.stamp = self.get_clock().now().to_msg()
            self.img_stop_line_pub.publish(img_msg_stop)
            self.get_logger().debug("정지선 이미지 퍼블리시 완료")
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 오류: {e}")
        """

        end_time = time.time() 
        # self.get_logger().info(f"stopline 실행 시간: {end_time - start_time:.3f} 초")

    def inverse_perspective_mapping(self, img):
        camera_matrix = np.array([[530.91193822,  0., 323.94406173, 0.],
                                  [0., 480.85427703, 260.42251396, 0.],
                                  [0., 0., 1., 0.]])
        
        world_x_max = 10.0
        world_x_min = 1.0
        world_y_max = 2
        world_y_min = -2

        world_x_interval = 0.05 / 2.0
        world_y_interval = 0.08 / 2.0

        output_width = int(np.ceil((world_y_max - world_y_min) / world_y_interval))
        output_height = int(np.ceil((world_x_max - world_x_min) / world_x_interval))
        
        print("(width, height) :", "(", output_width, ",",  output_height, ")")
        map_x, map_y = self.generate_direct_backward_mapping(
            world_x_min, world_x_max, world_x_interval, 
            world_y_min, world_y_max, world_y_interval, 
            extrinsic=self.RT, intrinsic=camera_matrix)
        
        output_image = cv2.remap(img, map_x, map_y, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
        return output_image
    
    def generate_direct_backward_mapping(self,
        world_x_min, world_x_max, world_x_interval, 
        world_y_min, world_y_max, world_y_interval, extrinsic, intrinsic):
        
        world_x_coords = np.arange(world_x_max, world_x_min, -world_x_interval)
        world_y_coords = np.arange(world_y_max, world_y_min, -world_y_interval)
        
        output_height = len(world_x_coords)
        output_width = len(world_y_coords)
        
        world_x_grid, world_y_grid = np.meshgrid(world_x_coords, world_y_coords)
        world_points = np.dstack([world_x_grid, world_y_grid, np.zeros_like(world_x_grid), np.ones_like(world_x_grid)])
        world_points = world_points.reshape(-1, 4).T

        points_c = intrinsic @ (extrinsic @ world_points)
        points_pixel = points_c / points_c[2]
        points_pixel = points_pixel[:2]
        
        map_x = points_pixel[0].reshape((output_height, output_width), order='F').astype(np.float32)
        map_y = points_pixel[1].reshape((output_height, output_width), order='F').astype(np.float32)

        return map_x, map_y

    def detect_stop_line_using_contours(self, binary_img, original_img, min_width_ratio=0.1, max_width_ratio=1, min_area=150):
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_width = binary_img.shape[1]
        valid_contours = []
        center_points = []
        # self.get_logger().debug(f"검출된 컨투어 개수: {len(contours)}")
        
        for contour in contours:
            # 면적 필터링 추가
            area = cv2.contourArea(contour)
            if area < min_area: # 너무 작은 컨투어는 무시
                continue
            
            x, y, width, height = cv2.boundingRect(contour)
            aspect_ratio = float(width) / height
            self.get_logger().info(f"Contour: x={x}, y={y}, width={width}, height={height}")

            if min_width_ratio * img_width < width < max_width_ratio * img_width and aspect_ratio > 0.4:
                valid_contours.append(contour)
                center_x = x + width / 2
                center_y = y + height / 2
                center_points.append((center_x, center_y))
                
                cv2.rectangle(original_img, (x, y), (x + width, y + height), (255, 0, 0), 2)
                cv2.drawContours(original_img, valid_contours, -1, (0, 255, 0), 2)

        # self.get_logger().debug(f"유효한 컨투어 수: {len(valid_contours)}")
        # self.get_logger().debug(f"컨투어 중심 좌표: {center_points}")
        return original_img, center_points
     
    
    def stopline(self, img):
        img_raw = img
        # BEV 변환 (원근 왜곡 보정)
        bev_img = self.bev(img_raw)
        # self.get_logger().debug(f"BEV 이미지 shape: {bev_img.shape}")
        
        # 대비 향상
        clahe_img = self.clahe(bev_img)
        # 흰색 영역 선택
        white_img = self.select_white(clahe_img)
        gray_img = cv2.cvtColor(white_img, cv2.COLOR_BGR2GRAY)
        
        ret, binary_img = cv2.threshold(gray_img, 220, 255, cv2.THRESH_BINARY)
        binary_img = binary_img[100:, :]
        bev_img = bev_img[100:, :]
       
        contour_img, center_points = self.detect_stop_line_using_contours(binary_img, bev_img)
        self.get_logger().info(f"검출된 정지선 중심 좌표: {center_points}")

        '''
        bgr_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        # result_img를 항상 할당
        result_img = np.hstack([bev_img, bgr_img])
        self.get_logger().info(f"result_img: {result_img.shape}")
        '''
        
        if center_points: # center_points가 비어 있지 않으면 실행
            pixel_coordinates = np.mean(np.array(center_points), axis=0) # 중심점들의 평균 위치 계산
            y_scale = 0.025

            # pixel_coordinates[1] 대신 pixel_coordinates의 두 번째 값 사용
            lane_world_x = y_scale * (bev_img.shape[0] - pixel_coordinates[1])
            lane_world_x += 4.0
            # print("y좌표 변환: ", lane_world_x)
            
            at_error = lane_world_x

            ate_msg = Float32()
            ate_msg.data = at_error
            self.ate_pub.publish(ate_msg)
            # self.get_logger().info(f"계산된 오차: {at_error:.3f}")
            '''
            # [✔ ADDED] 정지선 검출 시 텍스트 표시
            cv2.putText(
                result_img, 
                "Stop line detected!", 
                (10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (0, 0, 255), 2
            )
            '''
        else:
            self.get_logger().warn("정지선이 검출되지 않았습니다.")
 
        bgr_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        result_img = np.hstack([bev_img, bgr_img])
        # self.get_logger().info(f"result_img: {result_img.shape}")

        return result_img
    
    def bev(self, img):
        pts1 = np.float32([(250, 245), (410, 245), (620, 350), (30, 350)])
        pts2 = np.float32([(10, 10), (90, 10), (95, 360), (5, 360)])

        mtrx = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(img, mtrx, (100, 360))

        return result
        
    def clahe(self, img):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        Ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        Y, cr, cb = cv2.split(Ycrcb)
        Y_clahe = clahe.apply(Y)
        Ycrcb_clahe = cv2.merge((Y_clahe, cr, cb))
        res = cv2.cvtColor(Ycrcb_clahe, cv2.COLOR_YCrCb2BGR)

        return res

    def select_white(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        # 원래
        white_lower = np.array([20, 170, 0]) 
        white_upper = np.array([179, 255, 255])

        #칼라에서 값
        # white_lower = np.array([20, 200, 0])
        # white_upper = np.array([179, 255, 255])

        white_mask = cv2.inRange(hls, white_lower, white_upper)
        white_masked = cv2.bitwise_and(image, image, mask=white_mask)
        result_img = cv2.cvtColor(white_masked, cv2.COLOR_HLS2BGR)

        return result_img

def rotation_from_euler(roll=1., pitch=1., yaw=1.):
    """
    Get rotation matrix
    Args:
        roll, pitch, yaw:       In radians

    Returns:
        R:          [4, 4]
    """
    si, sj, sk = np.sin(roll), np.sin(pitch), np.sin(yaw)
    ci, cj, ck = np.cos(roll), np.cos(pitch), np.cos(yaw)

    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    R = np.identity(4)
    R[0, 0] = cj * ck
    R[0, 1] = sj * sc - cs
    R[0, 2] = sj * cc + ss
    R[1, 0] = cj * sk
    R[1, 1] = sj * ss + cc
    R[1, 2] = sj * cs - sc
    R[2, 0] = -sj
    R[2, 1] = cj * si
    R[2, 2] = cj * ci
    return R

def translation_matrix(vector):
    """
    Translation matrix

    Args:
        vector list[float]:     (x, y, z)

    Returns:
        T:      [4, 4]
    """
    M = np.identity(4)
    M[:3, 3] = vector[:3]
    return M
    
def main(args=None):
    rclpy.init(args=args)
    node = StopLineDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()