#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from autocar_utils.euler_from_quaternion import euler_from_quaternion
from autocar_utils.yaw_to_quaternion import yaw_to_quaternion
from autocar_utils.normalise_angle import normalise_angle
from .acados_setting_sp import acados_solver
from autocar_utils.utils import CubicSpline2D

from rviz_2d_overlay_msgs.msg import OverlayText
from visualization_msgs.msg import Marker, MarkerArray
from planning_msgs.msg import ModeState
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy

NX = 5  # 상태 변수 크기 (x, y, yaw, v, s)
NU = 2 # 제어 입력 크기 (delta , a)
T = 3.0  # 예측 시간 [s]
N = 30  # 예측 구간 [s]

class Control(Node):
    def __init__(self):
        super().__init__('control')

        # Publisher
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        # 시각화 Publisher
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/overlay', 10)
        self.global_path_pub = self.create_publisher(MarkerArray, '/autocar/global_path', qos_profile=qos_transient_local)
        self.mpc_predict_pub = self.create_publisher(MarkerArray, '/autocar/mpc_predict', qos_profile=qos_transient_local)
        self.mpc_ref_pub = self.create_publisher(MarkerArray, '/autocar/mpc_ref', qos_profile=qos_transient_local)

        # Subscriber
        self.localization_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.global_waypoints_sub = self.create_subscription(PoseArray, '/autocar/goals', self.global_waypoints_cb, 10)
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_cb, 10)

        self.local_waypoints_sub = self.create_subscription(PoseArray, '/autocar/local_goals', self.local_waypoints_cb, 10)

        self.map_origin_sub = self.create_subscription(PointStamped, '/map/origin', self.map_origin_cb, qos_transient_local)

        self.obstacle_sub = self.create_subscription(MarkerArray, '/obstacles/markers', self.obstacle_cb, 10)

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None
        self.s = None

        self.xs_global = []
        self.ys_global = []
        self.cubic_spline = None  

        self.xs_local = []
        self.ys_local = []
        self.cubic_spline_local = None

        self.lock = threading.Lock()
        self.control_frequency = 20.0 # HZ
        self.dt = T / N

        self.obs1_x = None
        self.obs1_y = None
        self.obs2_x = None
        self.obs2_y = None

        self.target_vel = 3.0  # 목표 속도 (m/s)
        self.steering_angle = 0.0
        self.velocity = 0.0
        
        # 이전 제어 입력 저장용 변수 (solver 실패 시 fallback용)
        self.prev_steering_angle = 0.0
        self.prev_velocity = 0.0
        self.fail_count = 0  # 실패 횟수 카운트
        
        # map 원점
        self.map_origin_x = None
        self.map_origin_y = None

        self.mode = 0
        self.mode_description = None

        # MPC Solver 초기화
        self.solver = acados_solver() 

        # 주기적인 제어 실행을 위한 타이머 설정
        self.timer_control = self.create_timer(1.0 / self.control_frequency, self.mpc_control)

    def map_origin_cb(self, msg):
        """ 
        맵 원점 정보 업데이트 
        """
        self.map_origin_x = msg.point.x
        self.map_origin_y = msg.point.y
        # self.get_logger().info(f"Map origin set to: ({self.map_origin_x}, {self.map_origin_y})")

    def vehicle_state_cb(self, msg):
        """
        차량 상태 업데이트 콜백
        """
        if self.map_origin_x is None or self.map_origin_y is None:
            self.get_logger().warn("Map origin 정보가 설정되지 않았습니다. 차량의 상태를 업데이트할 수 없습니다.")
            return
        
        self.lock.acquire()

        self.x = msg.pose.pose.position.x - self.map_origin_x
        self.y = msg.pose.pose.position.y - self.map_origin_y

        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.v = np.sqrt((msg.twist.twist.linear.x ** 2.0) + (msg.twist.twist.linear.y ** 2.0))
        if abs(self.v) < 0.001:
            self.v = 0.1
        self.yawrate = msg.twist.twist.angular.z

        if len(self.xs_global) > 0 or len(self.xs_local) > 0:
            self.calc_current_s()

        self.lock.release()

    def mode_cb(self, msg):
        """
        모드 상태 업데이트 콜백
        """
        self.mode = msg.current_mode
        self.mode_description = msg.description    

    def calc_current_s(self):
        """
        이진 탐색과 gradient descent를 사용한 s 값 탐색
        """
        if self.x is None or self.y is None:
            self.s = 0.0
            return
            
        if not hasattr(self.cubic_spline, 's') or len(self.cubic_spline.s) == 0:
            self.s = 0.0
            return
        
        total_length = self.cubic_spline.s[-1]
        
        # 1단계: 거친 탐색으로 대략적인 위치 찾기
        coarse_resolution = 1.0  # 1m 간격
        s_coarse = np.arange(0, total_length + coarse_resolution, coarse_resolution)
        
        min_distance = float('inf')
        best_s = 0.0
        
        for s_val in s_coarse:
            sx, sy = self.cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val
        
        # 2단계: 최적 지점 주변을 세밀하게 탐색
        search_range = 2.0  # 최적점 주변 ±2m 범위
        s_start = max(0, best_s - search_range)
        s_end = min(total_length, best_s + search_range)
        
        fine_resolution = 0.05  # 0.05m 간격으로 세밀 탐색
        s_fine = np.arange(s_start, s_end + fine_resolution, fine_resolution)
        
        for s_val in s_fine:
            sx, sy = self.cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val
        
        self.s = best_s
        return

    def global_waypoints_cb(self, path_msg):
        """
        global waypoints 콜백
        """

        self.xs_global, self.ys_global = [], []  # waypoint 리스트
        for node in path_msg.poses:
            self.xs_global.append(node.position.x)
            self.ys_global.append(node.position.y)

        # self.cubic_spline = CubicSpline2D(self.xs_global, self.ys_global) # waypoint를 보간한 CubicSpline2D 객체 생성
        self.make_cubic_spline()

        return 
    
    def local_waypoints_cb(self, path_msg):
        """
        local waypoints 콜백
        """
        self.xs_local, self.ys_local = [], []  # waypoint 리스트
        for node in path_msg.poses:
            self.xs_local.append(node.position.x)
            self.ys_local.append(node.position.y)

        # self.cubic_spline_local = CubicSpline2D(self.xs_local, self.ys_local)  # waypoint를 보간한 CubicSpline2D 객체 생성
        self.make_cubic_spline()

        return
    
    def make_cubic_spline(self):
        if self.mode == 0: # DRIVE 모드
            self.cubic_spline = CubicSpline2D(self.xs_global, self.ys_global)
        
        else: # MISSION 모드
            self.cubic_spline = CubicSpline2D(self.xs_local, self.ys_local)

    def obstacle_cb(self, msg):
        """ 
        장애물 위치 업데이트 
        """
        self.obs1_x = msg.markers[0].pose.position.x - self.map_origin_x
        self.obs1_y = msg.markers[0].pose.position.y - self.map_origin_y
        self.obs2_x = msg.markers[1].pose.position.x - self.map_origin_x
        self.obs2_y = msg.markers[1].pose.position.y - self.map_origin_y

        # print(f"obs1: ({self.obs1_x}, {self.obs1_y}), obs2: ({self.obs2_x}, {self.obs2_y})")

    def calc_ref_trajectory(self):
        """
        MPC 예측 step에 대한 refrecne trajectory 계산
        """
        
        xref = np.zeros((NX, N)) # reference x, y, yaw, v, s
        tan_vec = np.zeros((2, N)) # 접선 벡터 tx, ty

        if self.cubic_spline:
            current_s = self.s

            for i in range(N):
                s = current_s + self.dt * self.target_vel
                if s > self.cubic_spline.s[-1]:
                    s = self.cubic_spline.s[-1] - 0.01

                xref[0, i], xref[1, i] = self.cubic_spline.calc_position(s)
                xref[2, i] = self.cubic_spline.calc_yaw(s)
                xref[3, i] = self.target_vel
                xref[4, i] = s 

                tan_vec[0, i] = math.cos(self.cubic_spline.calc_yaw(s))
                tan_vec[1, i] = math.sin(self.cubic_spline.calc_yaw(s))

                current_s = s

        self.visualize_ref_trajectory(xref)

        return xref, tan_vec

    def mpc_control(self):
        """
        MPC 제어 루프
        """

        if self.x is None or self.y is None or self.yaw is None or self.v is None:
            self.get_logger().warn("차량 상태가 초기화되지 않았습니다.")
            return

        if self.xs_global == [] or self.ys_global == []:
            self.get_logger().warn("Global waypoints 데이터가 없습니다.")
            return
        
        self.calc_current_s()
        xref, tan_vec = self.calc_ref_trajectory()

        x0 = np.array([self.x, self.y, self.yaw, self.v, self.s])

        obs = np.array([self.obs1_x, self.obs1_y, self.obs2_x, self.obs2_y])

        self.solver.set(0, "x", x0)
        self.solver.constraints_set(0, "lbx", x0)
        self.solver.constraints_set(0, "ubx", x0)

        self.get_logger().info(f"Current state: {x0}")

        for i in range(N):
            self.solver.set(i, "p", np.hstack([xref[:5, i], tan_vec[:, i], obs]))
        self.solver.set(N, "p", np.hstack([xref[:5, -1], tan_vec[:, -1], obs]))

        status = self.solver.solve()
        if status != 0:
            self.fail_count += 1
            self.get_logger().error(f"MPC Solver failed with status {status}.")
            self.prev_steering_angle *= 0.98
            self.prev_velocity *= 0.98
            self.set_vehicle_command(self.prev_steering_angle, self.prev_velocity)
            return
        
        self.get_logger().info(f"tan_vec: {tan_vec}\
                               \n xref: {xref[:, 0]}, {xref[:, 1]}, {xref[:, 2]}, {xref[:, 3]}, {xref[:, 4]}")
        
        u_opt = self.solver.get(0, "u")
        x_opt = np.array([self.solver.get(i, "x") for i in range(N)])
        self.visualize_predicted_trajectory(x_opt)

        self.steering_angle = u_opt[0]
        self.velocity = x_opt[1, 3]

        # 이전 제어 입력 저장 (다음 실패 시 fallback용)
        self.prev_steering_angle = self.steering_angle
        self.prev_velocity = self.velocity

        # 차량에 제어 명령 전송
        self.set_vehicle_command(self.steering_angle, self.velocity)

        self.get_logger().info(f"cmd_steer: {self.steering_angle * 180.0 / np.pi:.2f} deg, cmd_vel: {self.velocity:.2f} m/s")

    def set_vehicle_command(self, steering_angle, velocity):
        """
        차량 명령 퍼블리시
        """

        cmd = AckermannDriveStamped()
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = steering_angle

        self.erp_pub.publish(cmd)


        self.publish_overlay_text()

        self.get_logger().info(f"속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg")



# _____________________________ visualization ______________________________#

    def publish_overlay_text(self):
        text_msg = OverlayText()
        text_msg.width = 500
        text_msg.height = 200
        text_msg.text_size = 15.0
        text_msg.line_width = 2

        text_msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5) # 배경색 (반투명 검정)
        text_msg.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # 글자색 (파란색)

        # 표시할 텍스트 설정
        text_msg.text = f"Velocity: {self.velocity:.2f}m/s \n Steer: {self.steering_angle * 180.0 / np.pi:.2f}deg\
            \n Fail Count: {self.fail_count}\
            \n Prev input: {self.prev_steering_angle * 180.0 / np.pi:.2f} deg, {self.prev_velocity:.2f} m/s \
            \n Mode: {self.mode} ({self.mode_description})"


        self.overlay_pub.publish(text_msg)

    def visualize_predicted_trajectory(self, x_opt):
        """
        Solver에서 예측된 경로를 시각화
        """
        marker_array = MarkerArray()

        for i in range(x_opt.shape[0]):  # x_opt의 각 점에 대해 반복
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "predicted_points"
            marker.id = i
            marker.type = Marker.ARROW  # 화살표로 표시
            marker.action = Marker.ADD

            # 위치 설정
            marker.pose.position.x = x_opt[i, 0]  # x 위치
            marker.pose.position.y = x_opt[i, 1]  # y 위치
            marker.pose.position.z = 0.0

            # 방향 설정 (yaw를 쿼터니언으로 변환)
            yaw = x_opt[i, 2]  # yaw 값
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
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_predict_pub.publish(marker_array)


    def visualize_ref_trajectory(self, xref):
        """
        xref를 시각화
        """
        marker_array = MarkerArray()

        for i in range(xref.shape[1]):  # xref의 각 점에 대해 반복
            marker = Marker()
            marker.header.frame_id = "map"
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
            marker.color.r = 1.0
            marker.color.g = 1.0  # 초록색
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_ref_pub.publish(marker_array)

# _____________________________ main function ______________________________#
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