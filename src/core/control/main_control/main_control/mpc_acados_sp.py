#!/usr/bin/env python3

import threading
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Header, Bool
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

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped

NX = 5  # 상태 변수 크기 (x, y, yaw, v, s)
NU = 2 # 제어 입력 크기 (delta , a)
T = 2.5  # 예측 시간 [s]
N = 25  # 예측 구간 [s]

class Control(Node):
    def __init__(self):
        super().__init__('control')

        # Publisher
        self.erp_pub = self.create_publisher(AckermannDriveStamped, '/erp/cmd_vel', 10)

        # 시각화 Publisher
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.overlay_pub = self.create_publisher(OverlayText, '/autocar/overlay', 10)
        self.mpc_predict_pub = self.create_publisher(MarkerArray, '/autocar/mpc_predict', qos_profile=qos_transient_local)
        self.mpc_ref_pub = self.create_publisher(MarkerArray, '/autocar/mpc_ref', qos_profile=qos_transient_local)

        # Subscriber
        self.localization_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.global_waypoints_sub = self.create_subscription(PoseArray, '/autocar/goals', self.global_waypoints_cb, 10)
        self.mode_sub = self.create_subscription(ModeState, '/mode_state', self.mode_cb, 10)

        self.local_waypoints_sub = self.create_subscription(Path, '/waypoints', self.local_waypoints_cb, 10) # parking 모드용 local waypoints

        self.map_origin_sub = self.create_subscription(PointStamped, '/map/origin', self.map_origin_cb, qos_transient_local)

        self.obstacle_sub = self.create_subscription(MarkerArray, '/obstacle_map', self.obstacle_cb, 10) # map 좌표로 변환된 장애물 토픽
        self.stopline_sub = self.create_subscription(Float64, '/stopline_distance', self.stopline_cb, 10) # 정지선 까지 거리
        # self.delivery_sub = self.create_subscription(Float64, '/delivery_distance', self.delivery_cb, 10) # 배달 지점 까지 거리
        self.delivery_sub = self.create_subscription(PoseArray, '/deliverysign_map', self.delivery_cb, 10) # 배달 지점 PoseArray 토픽

        self.reverse_flag_sub = self.create_subscription(Bool, '/reverse_flag', self.reverse_flag_cb, 10) # 후진 플래그
        self.person_detected_sub = self.create_subscription(Bool, '/person/detected', self.person_detected_cb, 10) # 사람 감지 플래그

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None
        self.s = None

        # [복구] Unwrapped Yaw 상태를 저장할 누적 변수 초기화
        self.yaw_unwrapped = None

        self.xs_global = []
        self.ys_global = []
        self.cubic_spline_global = None  

        self.xs_local = []
        self.ys_local = []
        self.cubic_spline_local = None

        self.lock = threading.Lock()
        self.control_frequency = 20.0 # HZ
        self.dt = T / N

        self.obs1_x = 1e4
        self.obs1_y = 1e4
        self.obs2_x = 1e4
        self.obs2_y = 1e4
        self.obs3_x = 1e4
        self.obs3_y = 1e4
        self.obs4_x = 1e4
        self.obs4_y = 1e4

        self.stopline_distance = 1e6
        self.delivery_distance = 1e6

        self.steering_angle = 0.0
        self.velocity = 0.0
        self.acc = 0.0

        # 이전 제어 입력 저장용 변수 (solver 실패 시 fallback용, 조향 변화율 비용용)
        self.prev_steering_angle = 0.0
        self.prev_velocity = 0.0
        self.fail_count = 0  # 실패 횟수 카운트
        
        # [롤백] Hot-Start 관련 변수 제거 (acados 내부 Hot-Start에 의존)
        
        # s 값 제약을 위한 변수들
        self.prev_s = 0.0  # 이전 s 값 저장
        self.s_tolerance = 30.0  # s 값이 역행할 수 있는 최대 허용 범위 (m)
        
        # map 원점
        self.map_origin_x = None
        self.map_origin_y = None

        # 모드 상태
        self.mode = 0 
        self.mode_description = "Drive"  
    
        # self.is_reverse = True
        self.is_reverse = False

        self.person_detected = False

        self.obs_type = 0 #미분류

        # 모드별 가중치 설정
        self.mode_weights = { # W_acc, W_steer, W_steer_rate, W_v, W_lag, W_con, W_yaw 
            0: np.array([1e-5, 0.1, 10.0, 6.0, 0.7, 1.5, 2.0]), # DRIVE
            1: np.array([1e-5, 0.1, 10.0, 6.0, 0.7, 3.0, 2.0]), # PAUSE
            2: np.array([0.05, 0.2, 2.0, 0.5, 1.0, 0.5, 0.1]), # OBSTACLE_STATIC (사용X)
            3: np.array([0.05, 0.2, 2.0, 0.5, 1.0, 0.5, 0.1]), # OBSTACLE_DYNAMIC (사용X)
            4: np.array([0.01, 0.2, 2.0, 0.5, 1.0, 0.5, 0.1]), # DELIVERY 
            5: np.array([0.1, 0.08, 5.0, 0.5, 0.7, 8.0, 10.0]), # PARKING
            6: np.array([0.05, 0.2, 2.0, 0.5, 1.0, 0.5, 0.1])  # RETURN (사용X)
        }       
        self.current_weights = self.mode_weights[self.mode]


        # 모드별 목표 속도 설정
        self.mode_target_vel = {
            0: 3.5,  # DRIVE
            1: 3.5,  # PAUSE
            2: 2.0,  # OBSTACLE_STATIC (사용X)
            3: 2.0,  # OBSTACLE_DYNAMIC (사용X)
            4: 1.0,  # DELIVERY
            5: 2.0,  # PARKING
            6: 3.0   # RETURN (사용X)
        }
        self.target_vel = self.mode_target_vel[self.mode]


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
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w) # self.yaw는 [-pi, pi]로 정규화됨

        # [복구] Unwrapped Yaw 업데이트 로직 추가: 연속적인 yaw 상태 유지
        if self.yaw_unwrapped is not None:
            # 현재 측정된 self.yaw를 [-pi, pi] 범위로 정규화
            wrapped_current_yaw = normalise_angle(self.yaw)
            
            # 이전 unwrapped yaw를 [-pi, pi]로 래핑하여 차이 계산의 기준점을 찾음
            yaw_diff = wrapped_current_yaw - normalise_angle(self.yaw_unwrapped)
            
            # 오차가 pi보다 크면 -2pi, -pi보다 작으면 +2pi를 더하여 보정 (가장 짧은 각도 차이)
            if yaw_diff > np.pi:
                yaw_diff -= 2 * np.pi
            elif yaw_diff < -np.pi:
                yaw_diff += 2 * np.pi
                
            # 누적된 yaw_unwrapped에 가장 짧은 각도 차이만큼 더하여 업데이트
            self.yaw_unwrapped = self.yaw_unwrapped + yaw_diff
        else:
            self.yaw_unwrapped = self.yaw # 초기값은 첫 측정된 yaw와 동일

        # Yaw가 None이면 unwrapped_yaw도 None이므로, 이 상태에서는 로그를 남기지 않음

        if self.is_reverse:
            self.v = -np.sqrt((msg.twist.twist.linear.x ** 2.0) + (msg.twist.twist.linear.y ** 2.0)) # 후진일 때 음수 속도 state 
        else:
            self.v = np.sqrt((msg.twist.twist.linear.x ** 2.0) + (msg.twist.twist.linear.y ** 2.0)) # 정상 주행일 때 양수 속도 state

        if abs(self.v) < 0.001:
            self.v = 0.1
            
        self.yawrate = msg.twist.twist.angular.z

        self.lock.release()

    def mode_cb(self, msg):
        """
        # ModeState.msg
        uint8 current_mode
        string description

        # Mode constants
        uint8 DRIVE=0
        uint8 PAUSE=1
        uint8 OBSTACLE_STATIC=2
        uint8 OBSTACLE_DYNAMIC=3
        uint8 DELIVERY=4
        uint8 PARKING=5
        uint8 RETURN=6
        uint8 UTURN=7
        uint8 GPS_OFF=8

        """
        # self.mode = msg.current_mode
        # self.mode_description = msg.description    
        if self.mode != msg.current_mode:
            self.mode = msg.current_mode
            self.mode_description = msg.description

            # 모드별 가중치 및 목표 속도 업데이트
            if self.mode in self.mode_weights:
                self.current_weights = self.mode_weights[self.mode]
                # self.get_logger().info(f"모드 변경: {self.mode_description}, 가중치 변경")
            else: 
                self.get_logger().warn(f"정의되지 않은 모드: {self.mode}, 기존 가중치 사용")

            if self.mode in self.mode_target_vel:
                self.target_vel = self.mode_target_vel[self.mode]
            else:
                self.get_logger().warn(f"정의되지 않은 모드: {self.mode}, 기존 목표 속도 사용")

    def reverse_flag_cb(self, msg):
        self.is_reverse = msg.data

    def person_detected_cb(self, msg):
        """
        사람 감지 플래그 콜백
        """
        person_detected = msg.data
        if person_detected:
            self.person_detected = True
            self.get_logger().info("사람 감지됨")
        else:
            self.person_detected = False

    def calc_current_s(self, _cubic_spline):
        """
        이진 탐색과 gradient descent를 사용한 s 값 탐색
        최근 s값 기준으로 일정 범위 안에서만 탐색
        """
        cubic_spline = _cubic_spline

        if self.x is None or self.y is None:
            self.s = 0.0
            return
            
        if not hasattr(cubic_spline, 's') or len(cubic_spline.s) == 0:
            self.s = 0.0
            return
        
        total_length = cubic_spline.s[-1]
        
        # s 탐색 범위를 이전 s 값 기준으로 제한
        search_start = max(0, self.prev_s - self.s_tolerance)
        search_end = min(total_length, self.prev_s + 50.0)  # 앞으로 최대 50m까지만 탐색
        
        # 1단계: 제한된 범위에서 거친 탐색
        coarse_resolution = 1.0  # 1m 간격
        s_coarse = np.arange(search_start, search_end + coarse_resolution, coarse_resolution)
        
        min_distance = float('inf')
        best_s = self.prev_s  # 기본값을 이전 s로 설정
        
        for s_val in s_coarse:
            # s 값이 유효한 범위 내에 있는지 확인
            if s_val < 0 or s_val > total_length:
                continue
            sx, sy = cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val
        
        # 2단계: 최적 지점 주변을 세밀하게 탐색
        search_range = 2.0  # 최적점 주변 ±2m 범위
        s_start = max(search_start, best_s - search_range)
        s_end = min(search_end, best_s + search_range)
        
        fine_resolution = 0.05  # 0.05m 간격으로 세밀 탐색
        s_fine = np.arange(s_start, s_end + fine_resolution, fine_resolution)
        
        for s_val in s_fine:
            # s 값이 유효한 범위 내에 있는지 확인
            if s_val < 0 or s_val > total_length:
                continue
            sx, sy = cubic_spline.calc_position(s_val)
            if sx is None or sy is None:
                continue
            distance = np.sqrt((self.x - sx)**2 + (self.y - sy)**2)
            if distance < min_distance:
                min_distance = distance
                best_s = s_val
        
        # s 값이 너무 크게 역행하는 것을 방지 
        if best_s < self.prev_s - self.s_tolerance:
            best_s = self.prev_s - self.s_tolerance
            self.get_logger().warn(f"S 값 역행 제한: {best_s:.2f} (이전: {self.prev_s:.2f})")

        if best_s < 0.0:
            best_s = 0.0
        
        self.prev_s = best_s  # 이전 s 값 업데이트
        self.s = best_s # 인스턴스 변수 self.s에 물리적 투영 s 값을 저장
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
        if self.map_origin_x is not None and self.map_origin_y is not None:
            for node in path_msg.poses:
                # self.xs_local.append(node.pose.position.x - self.map_origin_x)
                # self.ys_local.append(node.pose.position.y - self.map_origin_y)
                self.xs_local.append(node.pose.position.x)
                self.ys_local.append(node.pose.position.y)
        else:
            self.get_logger().warn("Map origin 정보가 설정되지 않았습니다. Local waypoints를 업데이트할 수 없습니다.")
            return
        
        # for node in path_msg.poses:
        #     self.xs_local.append(node.pose.position.x)
        #     self.ys_local.append(node.pose.position.y)

        # self.cubic_spline_local = CubicSpline2D(self.xs_local, self.ys_local)  # waypoint를 보간한 CubicSpline2D 객체 생성
        self.make_cubic_spline()

        return
    
    def make_cubic_spline(self):
        # Global waypoints로 스플라인 생성 (DRIVE/PAUSE/DELIVERY 모드용)
        if len(self.xs_global) >= 2 and len(self.ys_global) >= 2:
            self.cubic_spline_global = CubicSpline2D(self.xs_global, self.ys_global)
            self.get_logger().info(f"Global cubic spline 생성 완료: {len(self.xs_global)}개 포인트")
        
        # Local waypoints로 스플라인 생성 (PARKING 모드용)
        if len(self.xs_local) >= 2 and len(self.ys_local) >= 2:
            self.cubic_spline_local = CubicSpline2D(self.xs_local, self.ys_local)
            self.get_logger().info(f"Local cubic spline 생성 완료: {len(self.xs_local)}개 포인트")

    def obstacle_cb(self, msg):
        """ 
        obstacle_map_node에서 변환된 장애물 위치 업데이트 (이미 map 좌표계)
        최대 4개 장애물 지원 - 개수 변경 시 기존 정보 초기화
        """
        if len(msg.markers) == 0:
            # 장애물이 없을 때는 멀리 있는 가상 위치로 설정
            self.obs1_x = 1e4
            self.obs1_y = 1e4
            self.obs2_x = 1e4
            self.obs2_y = 1e4
            self.obs3_x = 1e4
            self.obs3_y = 1e4
            self.obs4_x = 1e4
            self.obs4_y = 1e4
            self.get_logger().debug("유효한 장애물이 없습니다. 가상 위치로 설정")
            return
        
        if self.map_origin_x is None or self.map_origin_y is None:
            self.get_logger().warn("Map origin 정보가 설정되지 않았습니다.")
            return

        try:
            obstacles = []
            
            # 각 CYLINDER 마커에서 위치 추출 (이미 map 좌표계)
            for marker in msg.markers:
                if marker.type == Marker.CYLINDER:
                    obs_x = marker.pose.position.x
                    obs_y = marker.pose.position.y
                    obstacles.append((obs_x, obs_y))

                    # 마커 색상 기반 장애물 종류 구분
                    # 파란색 = 드럼 / 노란색 = 자동차 / 흰색 = 미분류
                    if marker.color.b > 0.5:
                        self.obs_type = 1 # 드럼
                    elif marker.color.r > 0.5 and marker.color.g > 0.5:
                        self.obs_type = 2 # 자동차
                    else:
                        self.obs_type = 0 # 미분류
                    
                    self.get_logger().debug(f"장애물: ({obs_x:.2f}, {obs_y:.2f})")
            
            # 모든 장애물을 먼저 초기화**
            self.obs1_x = 1e4
            self.obs1_y = 1e4
            self.obs2_x = 1e4
            self.obs2_y = 1e4
            self.obs3_x = 1e4
            self.obs3_y = 1e4
            self.obs4_x = 1e4
            self.obs4_y = 1e4
            
            # 감지된 장애물 수에 따라 설정
            if len(obstacles) >= 4:
                self.obs1_x, self.obs1_y = obstacles[0]
                self.obs2_x, self.obs2_y = obstacles[1]
                self.obs3_x, self.obs3_y = obstacles[2]
                self.obs4_x, self.obs4_y = obstacles[3]
                self.get_logger().info(f"장애물 4개 감지 - obs1: ({self.obs1_x:.2f}, {self.obs1_y:.2f}), "
                                    f"obs2: ({self.obs2_x:.2f}, {self.obs2_y:.2f}), "
                                    f"obs3: ({self.obs3_x:.2f}, {self.obs3_y:.2f}), "
                                    f"obs4: ({self.obs4_x:.2f}, {self.obs4_y:.2f})")
            elif len(obstacles) == 3:
                self.obs1_x, self.obs1_y = obstacles[0]
                self.obs2_x, self.obs2_y = obstacles[1]
                self.obs3_x, self.obs3_y = obstacles[2]
                # obs4는 이미 1e4로 초기화됨
                self.get_logger().info(f"장애물 3개 감지 - obs1: ({self.obs1_x:.2f}, {self.obs1_y:.2f}), "
                                    f"obs2: ({self.obs2_x:.2f}, {self.obs2_y:.2f}), "
                                    f"obs3: ({self.obs3_x:.2f}, {self.obs3_y:.2f})")
            elif len(obstacles) == 2:
                self.obs1_x, self.obs1_y = obstacles[0]
                self.obs2_x, self.obs2_y = obstacles[1]
                # obs3, obs4는 이미 1e4로 초기화됨
                self.get_logger().info(f"장애물 2개 감지 - obs1: ({self.obs1_x:.2f}, {self.obs1_y:.2f}), "
                                    f"obs2: ({self.obs2_x:.2f}, {self.obs2_y:.2f})")
            elif len(obstacles) == 1:
                self.obs1_x, self.obs1_y = obstacles[0]
                # obs2, obs3, obs4는 이미 1e4로 초기화됨
                self.get_logger().info(f"장애물 1개 감지 - obs1: ({self.obs1_x:.2f}, {self.obs1_y:.2f})")
            else:
                # 모든 장애물이 이미 1e4로 초기화됨
                self.get_logger().warn("유효한 장애물이 없습니다. 가상 위치로 설정")

        except Exception as e:
            self.get_logger().error(f"장애물 처리 중 오류: {str(e)}")
            # 오류 발생시 안전한 기본값으로 설정
            self.obs1_x = 1e4
            self.obs1_y = 1e4
            self.obs2_x = 1e4
            self.obs2_y = 1e4
            self.obs3_x = 1e4
            self.obs3_y = 1e4
            self.obs4_x = 1e4
            self.obs4_y = 1e4
            
    def stopline_cb(self, msg):
        """ 
        정지선 위치 업데이트 
        """
        if msg.data is not None:
            self.stopline_distance = msg.data
        else:
            self.stopline_distance = 1e6

    # def delivery_cb(self, msg):
    #     """ 
    #     배달 지점 위치 업데이트 
    #     """
    #     if msg.data is not None:
    #         self.delivery_distance = msg.data
    #     else:
    #         self.delivery_distance = 1e6
    def delivery_cb(self, msg):
            """ 
            배달 지점 위치 업데이트 - PoseArray에서 첫 번째 포즈의 x,y 좌표를 사용하여 거리 계산
            """
            if len(msg.poses) == 0:
                self.delivery_distance = 1e6
                return
                
            if self.x is None or self.y is None:
                self.delivery_distance = 1e6
                return
                
            # 첫 번째 배달 지점 위치 추출
            delivery_x = msg.poses[0].position.x
            delivery_y = msg.poses[0].position.y
            
            # 현재 차량 위치와 배달 지점 사이의 유클리드 거리 계산
            self.delivery_distance = np.sqrt((self.x - delivery_x)**2 + (self.y - delivery_y)**2)
            
            
            self.get_logger().debug(f"배달 지점: ({delivery_x:.2f}, {delivery_y:.2f}), 거리: {self.delivery_distance:.2f}m")


    # calc_ref_trajectory 
    def calc_ref_trajectory(self, _cubic_spline):
        """
        MPC 예측 step에 대한 refrecne trajectory 계산
        """
        cubic_spline = _cubic_spline

        xref = np.zeros((NX, N)) # reference x, y, yaw, v, s
        tan_vec = np.zeros((2, N)) # 접선 벡터 tx, ty

        s_end = cubic_spline.s[-1] if cubic_spline else 0.0
        epsilon = 1e-6 # Robust Clamping을 위한 안전 여유

        if cubic_spline:
            current_s = self.s
            current_yaw = self.yaw_unwrapped if self.yaw_unwrapped is not None else self.yaw

            for i in range(N):
                # 다음 s 값을 먼저 계산
                s = current_s + self.dt * abs(self.target_vel) # v가 음수일 때도 s는 증가해야 함
                
                # [복구] s가 끝점을 넘어갔거나 같으면 클램핑
                if s >= s_end:
                    s = s_end - epsilon # 끝점에서 아주 작은 값만큼 뒤로 물림
                    target_vel_current = 0.0
                else:
                    # 장애물 감지 확인
                    obstacle_detected = (self.obs1_x < 1e3 or self.obs2_x < 1e3 or 
                                    self.obs3_x < 1e3 or self.obs4_x < 1e3)
                    
                    if obstacle_detected:
                        target_vel_current = min(abs(self.target_vel), 1.0)  # 장애물 감지 시 1.0 m/s로 제한
                    else:
                        target_vel_current = abs(self.target_vel)  # 정상 범위 내라면 원래 목표 속도 사용
                
                # 다음 iteration을 위해 current_s 업데이트
                current_s = s

                # 곡률에 따라 target_vel 조정 (주석 처리됨)
                speed_factor = 1.0
                curv_based_vel = target_vel_current * speed_factor

                xref[0, i], xref[1, i] = cubic_spline.calc_position(s)
                
                # Yaw Reference: 경로에서 얻은 yaw는 [-pi, pi] 범위임
                path_yaw = cubic_spline.calc_yaw(s)
                
                # [복구] Path Yaw를 현재 Unwrapped Yaw 주변으로 래핑하여 연속적인 Reference Yaw를 생성
                if current_yaw is not None:
                    # current_yaw(unwrapped) 주변의 가장 가까운 2pi 배수로 path_yaw를 보정
                    yaw_correction = round((current_yaw - path_yaw) / (2 * np.pi)) * 2 * np.pi
                    ref_yaw_unwrapped = path_yaw + yaw_correction
                else:
                    ref_yaw_unwrapped = path_yaw

                if self.is_reverse:
                    # 후진 시 yaw에 180도(pi)를 더한 값의 unwrapped 버전을 사용
                    xref[2, i] = ref_yaw_unwrapped + math.pi 
                else:
                    xref[2, i] = ref_yaw_unwrapped  # 전진 시 unwrapped yaw 사용

                xref[4, i] = s
                xref[3, i] = target_vel_current  # 항상 양수 속도의 reference 사용
                
                # 접선 벡터는 래핑된 yaw를 사용해도 됨 (방향만 필요)
                tan_vec[0, i] = math.cos(cubic_spline.calc_yaw(s))
                tan_vec[1, i] = math.sin(cubic_spline.calc_yaw(s))

        self.visualize_ref_trajectory(xref)

        return xref, tan_vec

    def mpc_control(self):
        """
        MPC 제어 루프
        """

        # 차량 상태 초기화 확인
        if self.x is None or self.y is None or self.yaw_unwrapped is None or self.v is None: # [복구] self.yaw_unwrapped로 변경
            self.get_logger().warn("차량 상태가 초기화되지 않았습니다.")
            return

        # 현재 모드에 따라 사용할 스플라인 결정
        if self.mode == 0 or self.mode == 1 or self.mode == 4 or self.mode == 8:  # DRIVE 모드 | PAUSE 모드 | DELIVERY 모드 / GPS off(0, 1, 4, 8)
            current_cubic_spline = self.cubic_spline_global
            if self.xs_global == [] or self.ys_global == []:
                self.get_logger().warn("Global waypoints 데이터가 없습니다.")
                return
            if current_cubic_spline is None:
                self.get_logger().warn("Global cubic spline이 초기화되지 않았습니다.")
                return
            
        else:  # PARKING 모드 포함 그 외의 모드 (2, 3, 5, 6)
            current_cubic_spline = self.cubic_spline_local
            if self.xs_local == [] or self.ys_local == []:
                self.get_logger().warn("Local waypoints 데이터가 없습니다.")
                return
            if current_cubic_spline is None:
                self.get_logger().warn("Local cubic spline이 초기화되지 않았습니다.")
                return

        # 현재 s 값 계산 (self.s 업데이트)
        self.calc_current_s(current_cubic_spline)
        
        # [복구] 현재 상태 벡터 x0 설정: self.yaw_unwrapped 사용
        x0 = np.array([self.x, self.y, self.yaw_unwrapped, self.v, self.s])

        # [롤백 유지] Hot-Start 로직 제거 (acados 내부 설정에 의존)

        # calc_ref_trajectory에 x0를 전달하여 동적 Reference Trajectory 계산
        xref, tan_vec = self.calc_ref_trajectory(current_cubic_spline)

        # 초기 상태 및 장애물 정보 설정
        obs = np.array([self.obs1_x, self.obs1_y, self.obs2_x, self.obs2_y, self.obs3_x, self.obs3_y, self.obs4_x, self.obs4_y])

        # [롤백 유지] 제어 입력 및 상태 변수는 0으로 초기화 (Cold-Start와 유사)
        u_opt = np.zeros((N, NU))  # 제어 입력 초기화 (delta, a)
        x_opt = np.zeros((N, NX))  # 상태 변수 초기화 (x, y, yaw, v, s)

        # Solver 초기 상태 변수 설정 (k=0의 상태는 현재 측정된 x0로 고정)
        self.solver.set(0, "x", x0)
        self.solver.constraints_set(0, "lbx", x0)
        self.solver.constraints_set(0, "ubx", x0)
        self.get_logger().info(f"Current state: {x0}")

        weights = self.current_weights

        if self.obs_type == 1: # 드럼
            r_safe = 1.3
        elif self.obs_type == 2: # 차량
            r_safe = 2.0
        else:
            r_safe = 0.0

        # MPC Solver에 파라미터 변수 전달
        for i in range(N):
            # [복구] Yaw Reference 래핑 로직 제거: xref[2, i]는 이미 calc_ref_trajectory에서 unwrapped version으로 준비됨.
            yaw_ref_unwrapped = xref[2, i]
            
            # 파라미터 설정
            params = np.hstack([
                np.array([xref[0, i], xref[1, i], yaw_ref_unwrapped, xref[3, i], xref[4, i]]),
                self.prev_steering_angle, # [롤백 유지] 이전 제어 입력(self.prev_steering_angle) 사용
                tan_vec[:, i],
                weights,
                obs
            ])
            self.solver.set(i, "p", params)
            self.solver.constraints_set(i, "lh", np.array([r_safe**2, r_safe**2, r_safe**2, r_safe**2]) )  # 최소 거리 제약 조건 설정
            self.solver.constraints_set(i, "uh", np.array([1e10, 1e10, 1e10, 1e10]) )  # 최대 거리 제약 조건 설정
        
        # Terminal Stage (N)
        yaw_ref_unwrapped_last = xref[2, -1]
        self.solver.set(N, "p", np.hstack([np.array([xref[0, -1], xref[1, -1], yaw_ref_unwrapped_last, xref[3, -1], xref[4, -1]]), self.prev_steering_angle, tan_vec[:, -1], weights, obs]))
        self.solver.constraints_set(N, "lh", np.array([r_safe**2, r_safe**2, r_safe**2, r_safe**2]) )  # 최소 거리 제약 조건 설정
        self.solver.constraints_set(N, "uh", np.array([1e10, 1e10, 1e10, 1e10]) )  # 최대 거리 제약 조건 설정   

        # Solver 실행, status 확인
        status = self.solver.solve()
        if status != 0:
            self.fail_count += 1
            self.get_logger().error(f"MPC Solver failed with status {status}.")
            self.prev_steering_angle *= 0.98
            self.prev_velocity *= 0.98  # solver failure 시 속도 감소
            self.set_vehicle_command(self.prev_steering_angle, self.prev_velocity) # 이전 제어 입력으로 차량에 입력
            return
        
        self.fail_count = 0  # 성공 시 fail count 초기화

        # Solver에서 최적화된 제어 입력, 상태 변수 추출
        # x_opt는 k=0부터 N까지 N+1개의 상태를 가져오도록 수정
        u_opt = np.array([self.solver.get(i, "u") for i in range(N)])
        x_opt = np.array([self.solver.get(i, "x") for i in range(N+1)]) 
        
        # [롤백 유지] 새로운 최적 해를 다음 루프를 위해 저장하는 로직 제거

        self.visualize_predicted_trajectory(x_opt[1:, :]) # k=1부터 N까지만 시각화

        # 제어 입력
        self.velocity = x_opt[1, 3]        # 속도 (v) -> 속도는 1step 뒤의 값을 사용
        self.steering_angle = u_opt[0, 0]   # 조향각 (delta) -> 조향각은 0step의 값을 사용

        self.acc = u_opt[0, 1] # 가속도 (a) (실제 cmd_vel로는 속도 값 이용, 디버그용)

        # 이전 제어 입력 저장 (다음 실패 시 fallback용)
        self.prev_steering_angle = self.steering_angle
        self.prev_velocity = self.velocity

        # s 값이 목표 지점에 도달했는지 확인 -> local path 활용 시 s의 끝에 도달했을 떄 속도를 0으로 설정
        remaining_distance = current_cubic_spline.s[-1] - self.s
        # if remaining_distance <= min(current_cubic_spline.s[-1]*0.2, 2.5): # s의 20%(path가 4.0m보다 짧을 경우) or 2.5m 이내에 도달했으면 정지
        if remaining_distance <= 1.7: # 1.7m 이내에 도달했으면 정지
            self.velocity = 0.0 # path의 끝점 근처에서 속도를 0으로 설정 -> 브레이크

        if self.mode == 1 and self.stopline_distance < 3.5: # PAUSE 모드이고 정지선 까지 거리가 3.5m 이내이면 정지
            self.velocity = 0.0

        if self.mode == 4 and self.delivery_distance < 1.5: # Delivery 모드이고 배달 지점 까지 거리가 4.0m 이내이면 정지
            self.velocity = 0.0

        if self.person_detected:
            self.velocity = 0.0

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

        # if self.mode == 1 and self.stopline_distance < 1.5:
        #     cmd.drive.speed = 0.0  # 정지선 근처에서는 속도를 0으로 설정

        self.erp_pub.publish(cmd)

        self.publish_overlay_text()
        self.get_logger().info(f"속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg")



# _____________________________ visualization ______________________________#

    def publish_overlay_text(self):
        text_msg = OverlayText()
        text_msg.width = 500
        text_msg.height = 350
        text_msg.text_size = 13.0
        text_msg.line_width = 2

        text_msg.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5) # 배경색 (반투명 검정)
        text_msg.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # 글자색 (파란색)

        # 표시할 텍스트 설정
        text_msg.text = f"cmd_vel: {self.velocity:.2f}m/s \n cmd_steer: {self.steering_angle * 180.0 / np.pi:.2f}deg\
            \n Acc: {self.acc:.2f}m/s² , v_err: {self.velocity - self.v:.2f}m/s\
            \n Fail Count: {self.fail_count}\
            \n Prev input: {self.prev_steering_angle * 180.0 / np.pi:.2f} deg, {self.prev_velocity:.2f} m/s \
            \n Mode: {self.mode} ({self.mode_description}) \
            \n Reverse: {self.is_reverse} \
            \n State: ({self.x:.2f}, {self.y:.2f}, {self.yaw_unwrapped:.2f}, {self.v:.2f}, {self.s:.2f}) \
            \n wrapped yaw: {self.yaw:.2f} \
            \n weights: {self.current_weights} \
            \n Obs1: ({self.obs1_x:.2f}, {self.obs1_y:.2f}), Obs2: ({self.obs2_x:.2f}, {self.obs2_y:.2f}), \
            Obs3: ({self.obs3_x:.2f}, {self.obs3_y:.2f}), Obs4: ({self.obs4_x:.2f}, {self.obs4_y:.2f}) \
            \n obs_type: {self.obs_type} \
            \n stopline: {self.stopline_distance:.2f}, delivery: {self.delivery_distance:.2f} " \

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
            # [복구 유지] Unwrapped Yaw를 시각화할 때는 [-pi, pi]로 래핑해야 Rviz에서 올바른 방향을 표시함
            yaw_wrapped = normalise_angle(x_opt[i, 2])
            quaternion = yaw_to_quaternion(yaw_wrapped)
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
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(marker)

        # 퍼블리시
        self.mpc_predict_pub.publish(marker_array)


    def visualize_ref_trajectory(self, xref):
        """
        xref를 시각화 (화살표 + 번호 텍스트)
        """
        marker_array = MarkerArray()

        for i in range(xref.shape[1]):  # xref의 각 점에 대해 반복
            # 화살표 마커
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = "xref_arrows"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW  # 화살표로 표시
            arrow_marker.action = Marker.ADD

            # 위치 설정
            arrow_marker.pose.position.x = xref[0, i]  # x 위치
            arrow_marker.pose.position.y = xref[1, i]  # y 위치
            arrow_marker.pose.position.z = 0.0

            # 방향 설정 (yaw를 쿼터니언으로 변환)
            # [복구 유지] Unwrapped Yaw를 시각화할 때는 [-pi, pi]로 래핑해야 Rviz에서 올바른 방향을 표시함
            yaw_wrapped = normalise_angle(xref[2, i])
            quaternion = yaw_to_quaternion(yaw_wrapped)
            arrow_marker.pose.orientation.x = quaternion.x
            arrow_marker.pose.orientation.y = quaternion.y
            arrow_marker.pose.orientation.z = quaternion.z
            arrow_marker.pose.orientation.w = quaternion.w

            # 크기 설정
            arrow_marker.scale.x = 0.3  # 화살표 길이
            arrow_marker.scale.y = 0.05  # 화살표 두께
            arrow_marker.scale.z = 0.05  # 화살표 두께

            # 색상 설정
            arrow_marker.color.r = 0.0 
            arrow_marker.color.g = 1.0 
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0  # 불투명

            # MarkerArray에 추가
            marker_array.markers.append(arrow_marker)

            # 번호 텍스트 마커
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "xref_numbers"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING  # 카메라를 향하는 텍스트
            text_marker.action = Marker.ADD

            # 텍스트 위치 (화살표보다 약간 위쪽에 표시)
            text_marker.pose.position.x = xref[0, i]
            text_marker.pose.position.y = xref[1, i]
            text_marker.pose.position.z = 0.3  # 지면에서 30cm 위

            # 텍스트 내용
            text_marker.text = str(i)

            # 텍스트 크기 설정
            text_marker.scale.z = 0.1  # 텍스트 높이

            # 텍스트 색상 설정 (흰색)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            # MarkerArray에 추가
            marker_array.markers.append(text_marker)

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