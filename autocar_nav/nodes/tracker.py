#!/usr/bin/env python3

import threading
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

from autocar_msgs.msg import Path2D, State2D
from autocar_nav import normalise_angle, yaw_to_quaternion

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # 퍼블리셔 생성
        self.tracker_pub = self.create_publisher(AckermannDriveStamped, '/autocar/autocar_cmd', 10)
        self.ct_error_pub = self.create_publisher(Float64, '/autocar/cte_error', 10)
        self.h_error_pub = self.create_publisher(Float64, '/autocar/he_error', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)

        # 서브스크라이버 생성
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path2D, '/autocar/path', self.path_cb, 10)
        self.target_vel_sub = self.create_subscription(Float64, '/autocar/target_velocity', self.target_vel_cb, 10)

        # ROS2 파라미터 설정
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', 50.0),  # 제어 업데이트 주기 (Hz)
                    ('control_gain', 1.0),  # Stanley 제어 게인
                    ('softening_gain', 1.0),  # 소프트닝 게인
                    ('yawrate_gain', 1.0),  # 조향 속도 게인
                    ('steering_limits', 0.95),  # 최대 조향각 제한 (rad)
                    ('centreofgravity_to_frontaxle', 1.438)  # 무게중심에서 전축까지의 거리 (m)
                ]
            )
            # 파라미터 값 가져오기
            self.frequency = float(self.get_parameter("update_frequency").value)
            self.k = float(self.get_parameter("control_gain").value)
            self.ksoft = float(self.get_parameter("softening_gain").value)
            self.kyaw = float(self.get_parameter("yawrate_gain").value)
            self.max_steer = float(self.get_parameter("steering_limits").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)
        
        except ValueError:
            raise Exception("ROS 파라미터가 누락되었습니다. 설정 파일을 확인하세요.")

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.target_vel = 0.0
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_idx = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.lock = threading.Lock()
        self.dt = 1 / self.frequency  # 제어 주기 계산

        # 주기적인 제어 실행을 위한 타이머 설정
        self.timer = self.create_timer(self.dt, self.timer_cb)

    # 타이머 콜백 함수 (주기적으로 Stanley 제어 실행)
    def timer_cb(self):
        self.stanley_control()

    # 차량 상태 콜백 함수 (현재 차량 위치 및 속도 업데이트)
    def vehicle_state_cb(self, msg):
        self.lock.acquire()
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))  # 속도 계산
        self.yawrate = msg.twist.w
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
            px = msg.poses[i].x
            py = msg.poses[i].y
            ptheta = msg.poses[i].theta
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta) 
        self.lock.release()

    # 목표 속도 수신 콜백 함수
    def target_vel_cb(self, msg):
        self.target_vel = msg.data

    # 목표 인덱스 계산 (경로에서 가장 가까운 점 찾기)
    def target_index_calculator(self):  
        fx = self.x + self.cg2frontaxle * -np.sin(self.yaw)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw)
        dx = [fx - icx for icx in self.cx]
        dy = [fy - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        front_axle_vec = [-np.cos(self.yaw + np.pi), -np.sin(self.yaw + np.pi)]
        self.crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
        self.heading_error = normalise_angle(self.cyaw[target_idx] - self.yaw - np.pi * 0.5)
        self.target_idx = target_idx

        # 참조 좌표 퍼블리시
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.cx[target_idx]
        pose.pose.position.y = self.cy[target_idx]
        pose.pose.orientation = yaw_to_quaternion(self.cyaw[target_idx])
        self.lateral_ref_pub.publish(pose)

    # Stanley 제어 알고리즘 실행
    def stanley_control(self):
        self.lock.acquire()
        crosstrack_term = np.arctan2((self.k * self.crosstrack_error), (self.ksoft + self.target_vel))
        heading_term = normalise_angle(self.heading_error)
        sigma_t = crosstrack_term + heading_term
        sigma_t = max(min(sigma_t, self.max_steer), -self.max_steer)
        self.set_vehicle_command(self.target_vel, sigma_t)
        self.lock.release()

    # 차량 명령을 퍼블리시하는 함수
    def set_vehicle_command(self, velocity, steering_angle):
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = velocity
        drive.drive.steering_angle = steering_angle
        self.tracker_pub.publish(drive)
        self.ct_error_pub.publish(Float64(data=self.crosstrack_error))
        self.h_error_pub.publish(Float64(data=self.heading_error))
        self.get_logger().info(f'속도: {velocity} | 조향각: {steering_angle}')

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
