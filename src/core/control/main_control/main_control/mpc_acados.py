#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from autocar_nav.normalise_angle import normalise_angle
from control.acados_setting import acados_solver
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
        
        # 시각화 Publisher
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/steering_angle', 10)
        self.ref_path_pub = self.create_publisher(Path, '/autocar/path', 10)
        self.mpc_ref_pub = self.create_publisher(MarkerArray, '/autocar/mpc_ref', 10)  
        self.mpc_predict_pub = self.create_publisher(MarkerArray, '/autocar/mpc_predict', 10)


        # Subscriber 생성
        self.localization_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.global_waypoints_sub = self.create_subscription(PoseArray, '/global_waypoints', self.global_waypoints_cb, 10)

        self.obstacle_sub = self.create_subscription(MarkerArray, '/obstacles/markers', self.obstacle_cb, 10)

        # 변수 초기화

        self.x = None
        self.y = None
        self.yaw = None
        self.vel = None
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.ck = []
        self.target_ind = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.lock = threading.Lock()
        self.dt = T / N  # 제어 주기 계산

        self.obs_x = None
        self.obs_y = None
        self.obs_a = None
        self.obs_b = None

        self.target_vel = 1.5  # 목표 속도 (m/s)
        self.steering_angle = 0.0
        self.velocity = 0.0

        # MPC Solver 초기화
        self.solver = acados_solver()  # 초기 상태 및 제어 입력 참조값

        # 주기적인 제어 실행을 위한 타이머 설정
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        self.mpc_control()

    def vehicle_state_cb(self, msg):
        self.lock.acquire()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.vel = np.sqrt((msg.twist.twist.linear.x**2.0) + (msg.twist.twist.linear.y**2.0))  # 속도 계산
        if abs(self.vel) < 1e-3:  # 임계값 설정 (1e-3 = 0.001 m/s)
            self.vel = 0.1

        self.yawrate = msg.twist.twist.angular.z
        # print(f'vel: {self.vel}')
        if self.cyaw:
            self.calc_nearest_index()
        self.lock.release()


    def global_waypoints_cb(self, path_msg):
        possible_change_direction = path_msg.header.frame_id

        path = {'x': None, 'y': None, 'yaw': None, 'k' : None, 's':None, 'csp':None}

        xs, ys = [], []
        for node in path_msg.poses:
            xs.append(node.position.x)
            ys.append(node.position.y)

        xs, ys, yaws, ks, s, csp = generate_target_course(xs, ys, step_size=0.2)
        path['x'] = xs
        path['y'] = ys
        path['yaw'] = yaws
        path['k'] = ks
        path['s'] = s
        path['csp'] = csp

        self.cx = xs
        self.cy = ys
        self.cyaw = yaws
        self.ck = ks
        
        path_msg = self.make_path_msg(path)
        self.ref_path_pub.publish(path_msg)


        return path, possible_change_direction
    
    def obstacle_cb(self, msg):
        self.obs_x = msg.markers[0].pose.position.x
        self.obs_y = msg.markers[0].pose.position.y
        self.obs_a = msg.markers[0].scale.x
        self.obs_b = msg.markers[0].scale.y
        print(f"obs_x: {self.obs_x}, obs_y: {self.obs_y}, obs_a: {self.obs_a}, obs_b: {self.obs_b}")
        


    def calc_ref_trajectory(self):
        """
        목표 궤적과 상태 참조값 계산
        """
        xref = np.zeros((NX, N))  # 상태 참조값 (x, y, yaw, v)
        uref = np.zeros((NU, N))  # 제어 입력 참조값 (steering, velocity)

        if self.cx and self.cy and self.cyaw and self.ck:
            current_index = self.target_ind
            
            # 디버깅을 위한 참조 경로 시각화 추가
            self.visualize_ref_trajectory(np.array([
                self.cx[current_index:current_index+N] if current_index+N < len(self.cx) else self.cx[current_index:],
                self.cy[current_index:current_index+N] if current_index+N < len(self.cy) else self.cy[current_index:],
                self.cyaw[current_index:current_index+N] if current_index+N < len(self.cyaw) else self.cyaw[current_index:],
                [self.target_vel] * min(N, len(self.cx) - current_index)
            ]))
            
            for i in range(N):
                # 곡률에 따른 동적 스텝 크기 계산
                curvature = abs(self.ck[current_index]) if current_index < len(self.ck) else 0
                
                # 곡률 기반 스텝 크기 계산
                # 곡률이 클수록 더 작은 스텝 (더 조밀한 포인트)
                base_step = 1.0
                # 곡률에 더 민감하게 반응하도록 계수 조정
                curvature_factor = 1.0 / (1.0 + 15.0 * curvature)  # 곡률이 클수록 작아짐 (8.0 -> 15.0)
                
                # 최종 스텝 크기 계산 (최소 0.5, 최대 2.0으로 제한)
                step = base_step * curvature_factor
                step = max(0.5, min(2.0, step))
                
                # 실제 인덱스 증가량 계산 (정수로 반올림)
                index_increment = max(1, round(step))
                
                # 다음 참조 인덱스 계산
                next_index = min(current_index + index_increment, len(self.cx) - 1)
                xref[:, i] = [self.cx[next_index], self.cy[next_index], self.cyaw[next_index], self.target_vel]                
                uref[:, i] = [0.0, self.target_vel]  # 초기 제어 입력 참조값

                # 현재 인덱스를 업데이트
                current_index = next_index

        # 디버깅: xref와 uref 출력
        print("xref:", xref)
        # print("uref:", uref)
        
        self.visualize_ref_trajectory(xref)  

        return xref, uref
    
    # def calc_ref_trajectory(self):
    #     """
    #     목표 궤적과 상태 참조값 계산
    #     """
    #     xref = np.zeros((NX, N))  # 상태 참조값 (x, y, yaw, v)
    #     uref = np.zeros((NU, N))  # 제어 입력 참조값 (steering, velocity)

    #     if self.cx and self.cy and self.cyaw:
    #         for i in range(N):
    #             ind = min(self.target_ind + i, len(self.cx) - 1)
    #             xref[:, i] = [self.cx[ind], self.cy[ind], self.cyaw[ind], self.target_vel]
    #             uref[:, i] = [0.0, self.target_vel]  # 초기 제어 입력 참조값

    #     # 디버깅: xref와 uref 출력
    #     print("xref:", xref)
    #     print("uref:", uref)

    #     self.visualize_ref_trajectory(xref)  

    #     return xref, uref    

    def calc_nearest_index(self):
        """
        차량의 현재 상태를 기반으로 경로에서 가장 가까운 인덱스를 계산하여 self.target_ind를 업데이트
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

        # 해당 인덱스와의 각도 차이 계산 (crosstrack error용)
        dxl = self.cx[ind] - self.x
        dyl = self.cy[ind] - self.y
        angle = normalise_angle(self.cyaw[ind] - math.atan2(dyl, dxl))

        # 각도가 음수이면, 거리의 부호를 반전 (crosstrack error 부호 결정)
        if angle < 0:
            mind *= -1
        
        self.crosstrack_error = mind
        
        # heading error 올바른 계산: 목표 방향 - 현재 방향
        self.heading_error = normalise_angle(self.cyaw[ind] - self.yaw)
        

        # self.target_ind 업데이트
        self.target_ind = ind


    def mpc_control(self):
        """
        MPC 제어 수행
        """
        if self.x is None or self.y is None or self.yaw is None or self.vel is None:
            print ("차량 상태가 초기화되지 않았습니다.")
            return

        if self.cx == [] or self.cy == [] or self.cyaw == []:
            print("경로 데이터가 없습니다.")
            return

        # 상태 및 제어 입력 참조값 계산
        self.calc_nearest_index()
        xref, uref = self.calc_ref_trajectory()
        u_prev = np.zeros((NU, N))  # 이전 제어 입력 초기화
        obs = np.array([self.obs_x, self.obs_y])

        # 초기 상태 설정
        x0 = np.array([self.x, self.y, self.yaw, self.vel])
        print("x0:", x0)  # 디버깅용 출력
        self.solver.set(0, "x", x0)
        self.solver.constraints_set(0, "lbx", x0)
        self.solver.constraints_set(0, "ubx", x0)

        # 장애물 정보 설정
        if self.obs_x is not None and self.obs_y is not None:
            obs = np.array([self.obs_x, self.obs_y])  # 장애물 
        else:
            obs = np.array([0.0, 0.0])  # 기본값 설정

        # 참조값 설정 (EXTERNAL cost에서는 p로 넘김)
        for i in range(N):
            self.solver.set(i, "p", np.hstack([xref[:, i], u_prev[:,i], obs]))
        self.solver.set(N, "p", np.hstack([xref[:, -1], u_prev[:, -1], obs]))  

        # Solver 실행
        status = self.solver.solve()
        if status != 0:
            self.get_logger().error(f"MPC Solver failed with status {status}")
            return

        # 최적화된 제어 입력 가져오기
        u_opt = self.solver.get(0, "u")
        for i in range(N):
            u_prev = np.array([self.solver.get(i, "u") for i in range(N)])  # 예측된 제어 입력
        self.steering_angle = u_opt[0]
        self.velocity = u_opt[1]

        # Solver에서 예측된 상태값 가져오기
        x_opt = np.array([self.solver.get(i, "x") for i in range(N)])  # 예측된 상태값
        self.visualize_predicted_trajectory(x_opt)
        print("x_opt:", x_opt)  # 디버깅용 출력

        print(f"u_opt: {u_opt}")

        # 차량 명령 퍼블리시
        self.set_vehicle_command(self.steering_angle, self.velocity)
        
    def set_vehicle_command(self, steering_angle, velocity):
        """
        차량 명령 퍼블리시
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        cmd_vel.angular.z = steering_angle

        cmd = AckermannDriveStamped()
        cmd.drive.speed = velocity
        cmd.drive.steering_angle = steering_angle

        self.erp_pub.publish(cmd)
        self.tracker_pub.publish(cmd_vel)
        self.ct_error_pub.publish(Float64(data=self.crosstrack_error))
        self.h_error_pub.publish(Float64(data=self.heading_error))

        self.publish_overlay_text()

        self.get_logger().info(f"속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg")

    
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
    
    
    def visualize_ref_trajectory(self, xref):
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

    def visualize_predicted_trajectory(self, x_opt):
        """
        Solver에서 예측된 경로를 시각화
        """
        marker_array = MarkerArray()

        for i in range(x_opt.shape[0]):  # x_opt의 각 점에 대해 반복
            marker = Marker()
            marker.header.frame_id = "world"
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
            marker.color.r = 1.0  # 빨간색
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_predict_pub.publish(marker_array)

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