#!/usr/bin/env python3

import threading
import numpy as np
import math
from collections import deque  
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from autocar_nav.normalise_angle import normalise_angle
from acados_setting import acados_solver
from autocar_nav.utils import generate_target_course

from rviz_2d_overlay_msgs.msg import OverlayText
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

NX = 4
NU = 2
T = 2.0
N = 20

class Control(Node):
    def __init__(self):
        super().__init__('control')

        # Publisher 생성
        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)
        self.ct_error_pub = self.create_publisher(Float64, '/autocar/cte', 10)
        self.h_error_pub = self.create_publisher(Float64, '/autocar/he', 10)
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/steering_angle', 10)

        self.ref_path_pub = self.create_publisher(Path, '/autocar/path', 10)
        self.mpc_ref_pub = self.create_publisher(MarkerArray, '/autocar/mpc_ref', 10)  


        # Subscriber 생성
        self.localization_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.global_waypoints_sub = self.create_subscription(PoseArray, '/global_waypoints', self.global_waypoints_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.accel = 0.0
        self.steer = 0.0
        self.accle_deq = deque(maxlen=5)
        self.vcmd = 0.0
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_ind = None
        self.crosstrack_error = 0.0
        self.heading_error = 0.0
        self.lock = threading.Lock()
        self.dt = T / N 

        self.target_vel = 1.5

        # MPC solver 초기화
        self.solver = acados_solver()

        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        # MPC solver 실행
        self.lock.acquire()
        if self.target_ind is not None:
            self.solve_mpc()
        self.lock.release()

    def vehicle_state_cb(self, msg):
        self.lock.acquire()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.v = np.sqrt((msg.twist.twist.linear.x**2.0) + (msg.twist.twist.linear.y**2.0))  # 속도 계산
        # if abs(self.v) < 1e-3:  # 임계값 설정 (1e-3 = 0.001 m/s)
        #     self.v = 0.1

        if self.cx :
            self.calc_nearest_index()
        self.lock.release()

    def global_waypoints_cb(self, path_msg):

        possible_change_direction = path_msg.header.frame_id

        path = {'x': None, 'y': None, 'yaw': None, 's':None, 'csp':None}

        xs, ys = [], []
        for waypoint in path_msg.poses:
            xs.append(waypoint.position.x)
            ys.append(waypoint.position.y)

        xs, ys, yaws, s, csp = generate_target_course(xs, ys, step_size=0.2)
        path['x'] = xs
        path['y'] = ys
        path['yaw'] = yaws
        path['s'] = s
        path['csp'] = csp

        self.cx = xs
        self.cy = ys
        self.cyaw = yaws
        
        path_msg = self.make_path_msg(path)
        self.ref_path_pub.publish(path_msg)

        return path, possible_change_direction
    
    def calc_ref(self):
        """
        reference trajectory calculation
        """
        xref = np.zeros((NX, N))
        uref = np.zeros((NU, N))

        if self.cx and self.cy and self.cyaw:
            for i in range(N):
                ind = min(self.target_ind + i, len(self.cx) -1)
                xref[:, i] = [self.cx[ind], self.cy[ind], self.cyaw[ind], self.target_vel]
                uref[:, i] = [0.0, 1.0]

        print("xref:", xref)
        print("uref:", uref)

        self.visualize_ref(xref)

        return xref, uref
    
    def calc_nearest_index(self):
        """
        nearest index calculation
        """
        if self.x is None or self.y is None or not self.cx or not self.cy or not self.cyaw:
            # 차량 상태나 경로 데이터가 없으면 업데이트하지 않음
            return 
        
        # 차량과 경로의 모든 점 사이의 거리 계산
        dx = [self.x - icx for icx in self.cx]
        dy = [self.y - icy for icy in self.cy]

        # 거리 계산 (제곱합)
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        # 최소 거리 찾기
        mind = min(d)
        ind = d.index(mind)  # 가장 가까운 인덱스 찾기

        # 실제 거리 값
        mind = math.sqrt(mind)
        print(f"mind: {mind}, ind: {ind}")  # 디버깅용 출력

        # 해당 인덱스와의 각도 차이 계산
        dxl = self.cx[ind] - self.x
        dyl = self.cy[ind] - self.y
        angle = normalise_angle(self.cyaw[ind] - math.atan2(dyl, dxl))


        # 각도가 음수이면, 거리의 부호를 반전
        if angle < 0:
            mind *= -1
        
        self.crosstrack_error = mind
        self.heading_error = angle

        # self.target_ind 업데이트
        self.target_ind = ind

    def solve_mpc(self):
        """
        MPC control calculation
        """
        if self.x is None or self.y is None or self.yaw is None or self.v is None:
            print ("차량 상태가 초기화되지 않았습니다.")
            return

        if self.cx == [] or self.cy == [] or self.cyaw == []:
            print("경로 데이터가 없습니다.")
            return
        
        # reference 계산
        self.calc_nearest_index()
        xref, uref = self.calc_ref()

        # 초기 상태 설정
        x0 = np.array([self.x, self.y, self.yaw, self.v])
        print("x0:", x0)
        self.solver.set(0, "x", x0)

        # reference 설정
        for i in range(N):
            self.solver.set(i, "yref", np.hstack((xref[:, i], uref[:, i])))
        self.solver.set(N, "yref", np.hstack((xref[:, -1], uref[:, -1])))
        
        # solver 실행
        status = self.solver.solve()
        if status != 0:
            self.get_logger().error(f"acados solve failed with status {status}")
            return
        
        u_opt = np.array(self.solver.get(0, "u"))
        x_opt = np.array(self.solver.get(0, "x"))

        self.steer = u_opt[0]
        self.accel = u_opt[1]

        self.accle_deq.append(self.accel)

        print(f"u_opt: {u_opt}")

        self.set_vehicle_command()

    def calc_vcmd(self):
        accel_avg = np.mean(self.accle_deq)
        self.vcmd = self.v + accel_avg * self.dt            

    def set_vehicle_command(self):
        """
        차량 제어 명령 설정
        """
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = self.steer
        cmd.drive.steering_angle_velocity = self.vcmd

        self.erp_pub.publish(cmd)
        self.ct_error_pub.publish(Float64(data=self.crosstrack_error))
        self.h_error_pub.publish(Float64(data=self.heading_error))

        self.get_logger().info(f"속도: {self.vcmd:.2f} m/s | 조향각: {self.steer * 180.0 / np.pi:.2f} deg")

    def publish_overlay_text(self):
        text_msg = OverlayText()
        text_msg.width = 500
        text_msg.height = 200
        text_msg.text_size = 15.0
        text_msg.line_width = 2

        text_msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5) # 배경색 (반투명 검정)
        text_msg.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # 글자색 (파란색)

        # 표시할 텍스트 설정
        text_msg.text = f"Accel: {self.accel:.2f}m/s^2\n Velocity: {self.vcmd:.2f}m/s \n Steer: {self.steer * 180.0 / np.pi:.2f}deg\
            \n CTE: {self.crosstrack_error:.2f} m \n HE: {self.heading_error * 180.0 / np.pi:.2f} deg"

        self.overlay_pub.publish(text_msg)

    def make_path_msg(self, path):
        
        path_x = path['x']
        path_y = path['y']
        path_yaw = path['yaw']

        ways = Path()
        ways.header = Header(frame_id='world', stamp=self.get_clock().now().to_msg())

        for i in range(len(path_x)-1):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "world"
            pose.pose.position.x = path_x[i]
            pose.pose.position.y = path_y[i]

            yaw = path_yaw[i]

            quaternion = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = quaternion.x
            pose.pose.orientation.y = quaternion.y
            pose.pose.orientation.z = quaternion.z
            pose.pose.orientation.w = quaternion.w

            ways.poses.append(pose)
        return ways
    
    
    def visualize_ref(self, xref):
        """
        xref를 시각화
        """
        marker_array = MarkerArray()

        for i in range(xref.shape[1]):  # xref의 각 점에 대해 반복
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "xref_points"
            marker.id = i
            marker.type = Marker.ARROW  # 화살표로 표시
            marker.action = Marker.ADD

            # 위치 설정
            marker.pose.position.x = xref[0, i]  # x 위치
            marker.pose.position.y = xref[1, i]  # y 위치
            marker.pose.position.z = 0.0

            # 방향 설정 (yaw를 쿼터니언으로 변환)
            yaw = xref[2, i]  # yaw 값
            quaternion = yaw_to_quaternion(yaw)
            marker.pose.orientation.x = quaternion.x
            marker.pose.orientation.y = quaternion.y
            marker.pose.orientation.z = quaternion.z
            marker.pose.orientation.w = quaternion.w

            # 크기 설정
            marker.scale.x = 0.3  # 화살표 길이
            marker.scale.y = 0.05  # 화살표 두께
            marker.scale.z = 0.05  # 화살표 두께

            # 색상 설정
            marker.color.r = 0.0
            marker.color.g = 1.0  # 초록색
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_ref_pub.publish(marker_array)

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    try:
        control = Control()
        rclpy.spin(control)
    finally:
        control.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()














    