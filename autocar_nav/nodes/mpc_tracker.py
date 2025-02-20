#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from autocar_msgs.msg import Path2D, State2D
from autocar_nav import normalise_angle, yaw_to_quaternion
from autocar_nav.mpc import State, calc_ref_trajectory, iterative_linear_mpc_control, update_state, calc_nearest_index

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # 퍼블리셔 생성
        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.steer_viz_pub = self.create_publisher(Marker, '/autocar/viz_steer', 10)
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        self.ct_error_pub = self.create_publisher(Float64, '/autocar/cte_error', 10)
        self.h_error_pub = self.create_publisher(Float64, '/autocar/he_error', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)

        # 서브스크라이버 생성
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path2D, '/autocar/path', self.path_cb, 10)

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
        self.frequency = 50.0
        self.dt = 1 / self.frequency  # 제어 주기 계산

        # 주기적인 제어 실행을 위한 타이머 설정
        self.timer = self.create_timer(self.dt, self.timer_cb)

    # 타이머 콜백 함수 (주기적으로 MPC 제어 실행)
    def timer_cb(self):
        self.mpc_control()

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

        self.get_logger().info(f'속도: {velocity:.2f} m/s | 조향각: {steering_angle:.2f} rad')
        self.get_logger().info(f'CTE: {self.crosstrack_error:.2f} m | HE: {self.heading_error:.2f} rad')

    def publish_steering_marker(self, steering_angle):
        marker = Marker()
        marker.header.frame_id = "base_link"  # 차량 기준 프레임
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "steering_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 시작점 (차량 중심)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # 조향각을 반영한 방향 설정
        marker.pose.orientation = yaw_to_quaternion(steering_angle)

        # 화살표 크기 설정
        marker.scale.x = 1.0  # 길이
        marker.scale.y = 0.05  # 화살표 두께
        marker.scale.z = 0.05

        # 색상 설정 (파란색)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # 투명도

        self.steer_viz_pub.publish(marker)


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