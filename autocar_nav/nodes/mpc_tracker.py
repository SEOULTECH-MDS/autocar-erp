#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped, Polygon, PolygonStamped, Point32, Twist
from rclpy.node import Node
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA

from autocar_nav import normalise_angle, yaw_to_quaternion
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.mpc import State, calc_ref_trajectory, iterative_linear_mpc_control, update_state, calc_nearest_index

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # 퍼블리셔 생성
        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/steering_angle', 10)
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        self.ct_error_pub = self.create_publisher(Float64, '/autocar/cte', 10)
        self.h_error_pub = self.create_publisher(Float64, '/autocar/he', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)

        self.state_prediction_pub = self.create_publisher(MarkerArray, '/autocar/state_prediction', 10)

        # 서브스크라이버 생성
        self.localisation_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path, '/autocar/path', self.path_cb, 10)

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.target_vel = 10.0 / 3.6
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_ind = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.lock = threading.Lock()
        self.frequency = 5.0
        self.dt = 1 / self.frequency  # 제어 주기 계산

        # 주기적인 제어 실행을 위한 타이머 설정
        self.timer = self.create_timer(self.dt, self.timer_cb)

    # 타이머 콜백 함수 (주기적으로 MPC 제어 실행)
    def timer_cb(self):
        self.mpc_control()

    # 차량 상태 콜백 함수 (현재 차량 위치 및 속도 업데이트)
    def vehicle_state_cb(self, msg):
        self.lock.acquire()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.vel = np.sqrt((msg.twist.twist.linear.x**2.0) + (msg.twist.twist.linear.y**2.0))  # 속도 계산
        self.yawrate = msg.twist.twist.angular.z
        if self.cyaw:
            self.target_index_calculator()
        self.lock.release()

    # 경로 데이터 수신 콜백 함수
    def path_cb(self, msg):
        self.lock.acquire()
        self.cx = []
        self.cy = []
        self.cyaw = []
        for i in range(len(msg.poses)):
            px = msg.poses[i].pose.position.x
            py = msg.poses[i].pose.position.y
            quaternion = msg.poses[i].pose.orientation
            ptheta = euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta) 
        self.lock.release()

    # 목표 인덱스 계산 (경로에서 가장 가까운 점 찾기)
    def target_index_calculator(self):  
        state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.vel)
        ind, mind = calc_nearest_index(state, self.cx, self.cy, self.cyaw, self.target_ind if self.target_ind is not None else 0)
        self.target_ind = ind

        dxl = self.cx[ind] - self.x
        dyl = self.cy[ind] - self.y

        angle = normalise_angle(self.cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        self.crosstrack_error = mind
        self.heading_error = normalise_angle(self.cyaw[ind] - self.yaw - np.pi * 0.5)

        # 참조 좌표 퍼블리시
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.cx[ind]
        pose.pose.position.y = self.cy[ind]
        pose.pose.orientation = yaw_to_quaternion(self.cyaw[ind])
        self.lateral_ref_pub.publish(pose)

    # MPC 제어 알고리즘 실행
    def mpc_control(self):
        self.lock.acquire()

        if self.target_ind is None or len(self.cx) == 0:
            self.lock.release()
            return


        state = State(x=self.x, y=self.y, yaw=self.yaw, v=self.vel)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        xref, target_ind, dref = calc_ref_trajectory(state, self.cx, self.cy, self.cyaw, [0]*len(self.cx), [self.target_vel]*len(self.cx), 1.0, self.target_ind)
        ov, od, ox, oy, oyaw, ov_state = iterative_linear_mpc_control(xref, x0, dref, ov=None, od=None)

        if ov is not None and od is not None:
            state = update_state(state, ov[0], od[0])
            self.set_vehicle_command(state.v, od[0])

            self.publish_vehicle_footprints(xref)

        self.lock.release()

    # 차량 명령을 퍼블리시하는 함수
    def set_vehicle_command(self, velocity, steering_angle):
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity  # 속도 설정
        cmd_vel.angular.z = steering_angle  # 조향 각도 설정


        cmd = AckermannDriveStamped()
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = steering_angle
        
        self.erp_pub.publish(cmd)
        self.tracker_pub.publish(cmd_vel)  # /autocar/cmd_vel 퍼블리시
        self.ct_error_pub.publish(Float64(data=self.crosstrack_error))
        self.h_error_pub.publish(Float64(data=self.heading_error))

        self.publish_steering_marker(steering_angle)
        self.publish_steering_text(steering_angle)

        self.get_logger().info(f'속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg')
        self.get_logger().info(f'CTE: {self.crosstrack_error:.2f} m | HE: {self.heading_error * 180.0 / np.pi:.2f} deg')
    
    def publish_steering_text(self, steering_angle):
        text_msg = OverlayText()
        text_msg.width = 300
        text_msg.height = 100
        text_msg.text_size = 20.0
        text_msg.line_width = 2

        # 배경색 (반투명 검정)
        text_msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)

        # 글자색 (파란색)
        text_msg.fg_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

        # 표시할 텍스트 설정
        text_msg.text = f"Steering Angle: {steering_angle * 180.0 / np.pi:.2f} deg"

        self.overlay_pub.publish(text_msg)
    

    # ======= 예측 상태 시각화 부분 (추가 필요) =======
    def publish_vehicle_footprints(self, xref, frame_id="world"):

        marker_array = MarkerArray()

        for i, state in enumerate(xref.T):
            x, y, v, yaw = map(float, state)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "vehicle_footprint"
            marker.id = i
            marker.type = Marker.CUBE  # CUBE 사용하여 직사각형 면으로 표현
            marker.action = Marker.ADD
            marker.scale.x = 1.5  # 차량 길이
            marker.scale.y = 1.0  # 차량 너비
            marker.scale.z = 0.05  # 두께 (0.1 정도로 설정)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 반투명

            # 위치 및 회전 설정
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0

            q = yaw_to_quaternion(yaw)  # Yaw를 쿼터니언으로 변환
            marker.pose.orientation.x = q.x
            marker.pose.orientation.y = q.y
            marker.pose.orientation.z = q.z
            marker.pose.orientation.w = q.w

            marker_array.markers.append(marker)

        self.state_prediction_pub.publish(marker_array)

    def get_rectangle_corners(self, x, y, yaw):
        # 차량 중심에서 직사각형의 네 꼭짓점 계산
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # 로컬 좌표계에서의 꼭짓점 (차량 중심이 원점)
        length = 1.5
        width = 1.0
        half_length = length / 2
        half_width = width / 2
        corners_local = np.array([
            [half_length, half_width],   # 오른쪽 앞
            [half_length, -half_width],  # 오른쪽 뒤
            [-half_length, -half_width], # 왼쪽 뒤
            [-half_length, half_width]   # 왼쪽 앞
        ])
        
        # 회전 변환 및 중심 이동
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        corners = corners_local @ rotation_matrix.T + np.array([x, y])
        
        # 닫힌 다각형을 위해 첫 점을 마지막에 추가
        corners = np.vstack([corners, corners[0]])

        return corners
    # ===========================================


# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    try:
        path_tracker = PathTracker()
        rclpy.spin(path_tracker)
    finally:
        path_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()