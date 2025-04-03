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
from scipy.spatial import KDTree

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

        # 서브스크라이버 생성
        self.localisation_sub = self.create_subscription(Odometry, '/autocar/location', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path, '/autocar/path', self.path_cb, 10)
    

        # 변수 초기화
        self.x = None
        self.y = None
        self.yaw = None
        self.target_vel = 2.0  # 초기 목표 속도를 낮게 설정 (약 7.2km/h)
        self.min_vel = 1.0  # 최소 속도 설정 (1.8km/h)
        self.max_vel = 5.0  # 최대 속도 설정 (18km/h)
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.target_ind = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.lock = threading.Lock()
        self.frequency = 20.0  # 제어 주기를 더 높게 설정
        self.dt = 1 / self.frequency

        # 제어 관련 파라미터 조정
        self.max_steering_angle = np.pi/6  # 30도로 제한
        self.max_steering_change = np.pi/12  # 15도로 제한
        self.prev_steering = 0.0
        
        # 초기 제어 상태
        self.control_initialized = False
        self.start_time = None
        self.velocity = 0.0
        self.steering_angle = 0.0

        # 주기적인 제어 실행을 위한 타이머 설정 (누락된 부분 추가)
        self.timer = self.create_timer(self.dt, self.timer_cb)

        # 곡률 계산 관련 변수 추가
        self.curvatures = []
        self.max_curvature = 0.5  # 최대 곡률 제한
        self.min_radius = 2.0     # 최소 회전 반경
        self.max_lateral_accel = 2.0  # 최대 횡방향 가속도 [m/s^2]
        self.prediction_horizon = 10  # 예측 구간
        
        # 시각화를 위한 퍼블리셔 추가
        self.curvature_pub = self.create_publisher(MarkerArray, '/autocar/curvature', 10)
        self.reference_point_pub = self.create_publisher(Marker, '/autocar/reference_point', 10)

        # 도로 폭 관련 변수 추가
        self.road_width = 3.0  # 도로 폭 [m]
        self.safety_margin = 0.5  # 안전 마진 [m]
        self.left_boundary = []  # 좌측 경계선
        self.right_boundary = []  # 우측 경계선
        
        # 경계선 시각화를 위한 퍼블리셔
        self.boundary_pub = self.create_publisher(MarkerArray, '/autocar/road_boundaries', 10)

    # 타이머 콜백 함수 (주기적으로 MPC 제어 실행)
    def timer_cb(self):
        """타이머 콜백 함수"""
        try:
            # 초기 상태에서는 기본 명령 발행
            if not self.control_initialized and self.x is not None and self.y is not None:
                self.velocity = self.min_vel
                self.steering_angle = 0.0
                self.set_vehicle_command(self.velocity, self.steering_angle)
                self.get_logger().info('초기 제어 명령 발행')
            
            # MPC 제어 실행
            self.mpc_control()
            
        except Exception as e:
            self.get_logger().error(f'타이머 콜백 에러: {str(e)}')
            # 에러 발생 시 안전한 정지 명령 발행
            self.set_vehicle_command(0.0, 0.0)

    # 차량 상태 콜백 함수 (현재 차량 위치 및 속도 업데이트)
    def vehicle_state_cb(self, msg):
        self.lock.acquire()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.vel = np.sqrt((msg.twist.twist.linear.x**2.0) + (msg.twist.twist.linear.y**2.0))  # 속도 계산
        self.yawrate = msg.twist.twist.angular.z
        print(f'vel: {self.vel}')
        if self.cyaw:
            self.target_index_calculator()
        self.lock.release()

    # def encoder_speed_cb(self, encoder_msg):
    #     self.vel = encoder_msg.data
    
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
        # 가장 가까운 인덱스를 계산하고, self.target_ind를 갱신
        ind, mind = calc_nearest_index(state, self.cx, self.cy, self.cyaw)
        self.target_ind = ind  # 새로 계산된 인덱스를 self.target_ind에 저장

        dxl = self.cx[ind] - self.x
        dyl = self.cy[ind] - self.y

        angle = normalise_angle(self.cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        self.crosstrack_error = mind
        # self.heading_error = normalise_angle(self.cyaw[ind] - self.yaw - np.pi * 0.5)
        self.heading_error = normalise_angle(self.cyaw[ind] - self.yaw)

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
        try:
            if self.target_ind is None or len(self.cx) == 0:
                self.lock.release()
                return

            # 초기 제어 단계 (부드러운 시작을 위해)
            if not self.control_initialized:
                self.control_initialized = True
                self.start_time = self.get_clock().now()
                self.velocity = self.min_vel
                self.steering_angle = 0.0
                self.set_vehicle_command(self.velocity, self.steering_angle)
                self.lock.release()
                return

            current_time = self.get_clock().now()
            if self.start_time is not None:
                time_diff = (current_time - self.start_time).nanoseconds / 1e9
                if time_diff < 2.0:  # 처음 2초 동안
                    # 부드럽게 속도 증가
                    self.velocity = min(self.min_vel + time_diff * 0.5, self.target_vel)
                    self.set_vehicle_command(self.velocity, self.steering_angle)
                    self.lock.release()
                    return

            state = State(x=self.x, y=self.y, yaw=self.yaw, v=max(self.vel, self.min_vel))
            x0 = [state.x, state.y, state.v, state.yaw]

            # 곡률 계산
            path_points = np.array(list(zip(self.cx, self.cy)))
            self.curvatures = self.calculate_curvature(path_points)
            
            # 곡률 기반 속도 프로파일링 수정
            target_velocities = []
            for curvature in self.curvatures:
                if abs(curvature) < 0.01:  # 직선 구간
                    target_velocities.append(min(self.target_vel, self.max_vel))
                else:
                    # 곡률에 따른 최대 속도 계산 (더 보수적인 값 사용)
                    max_vel = np.sqrt(self.max_lateral_accel / max(abs(curvature), 0.01))
                    target_vel = min(max_vel, self.target_vel)
                    target_vel = max(target_vel, self.min_vel)  # 최소 속도 보장
                    target_velocities.append(target_vel)

            # 참조 궤적 계산
            xref, target_ind, dref = calc_ref_trajectory(
                state, self.cx, self.cy, self.cyaw, 
                target_velocities, target_velocities, 1.0
            )

            # MPC 제어 입력 계산
            try:
                ov, od, ox, oy, oyaw, ov_state = iterative_linear_mpc_control(
                    xref, x0, dref, ov=None, od=None
                )
            except Exception as e:
                self.get_logger().error(f'MPC 최적화 실패: {str(e)}')
                # 최적화 실패 시 현재 속도 유지하고 조향만 보정
                if abs(self.heading_error) > np.pi/6:  # 30도 이상 틀어진 경우
                    self.steering_angle = np.clip(self.heading_error, -self.max_steering_angle, self.max_steering_angle)
                self.set_vehicle_command(self.velocity, self.steering_angle)
                self.lock.release()
                return

            if ov is not None and od is not None and ox is not None:
                # 조향각 제한 및 변화율 제한 적용
                steering_angle = np.clip(od[0], -self.max_steering_angle, self.max_steering_angle)
                steering_change = steering_angle - self.prev_steering
                if abs(steering_change) > self.max_steering_change:
                    steering_angle = self.prev_steering + np.sign(steering_change) * self.max_steering_change
                
                # 속도 제한 적용
                velocity = np.clip(ov[0], self.min_vel, self.max_vel)  # 속도 범위 제한
                
                # Cross Track Error가 큰 경우 속도 감소
                if abs(self.crosstrack_error) > 1.0:
                    velocity = max(self.min_vel, velocity * (1.0 - min(abs(self.crosstrack_error) / 5.0, 0.5)))
                
                # Heading Error가 큰 경우 속도 감소
                if abs(self.heading_error) > np.pi/6:  # 30도 이상
                    velocity = max(self.min_vel, velocity * (1.0 - min(abs(self.heading_error) / np.pi, 0.5)))
                
                # 제어 입력 적용
                state = update_state(state, velocity, steering_angle)
                self.set_vehicle_command(velocity, steering_angle)
                self.velocity = velocity
                self.steering_angle = steering_angle
                self.prev_steering = steering_angle

            # 경로 포인트로부터 도로 경계선 계산
            path_points = np.array(list(zip(self.cx, self.cy)))
            self.left_boundary, self.right_boundary = self.calculate_road_boundaries(path_points)
            
            # 시각화 업데이트
            self.publish_visualization()
            self.publish_boundary_visualization()

        finally:
            self.lock.release()

    # 차량 명령을 퍼블리시하는 함수
    def set_vehicle_command(self, velocity, steering_angle):
        # Twist 메시지 생성
        cmd_vel = Twist()
        cmd_vel.linear.x = float(velocity)  # 명시적으로 float로 변환
        cmd_vel.angular.z = float(steering_angle)  # 명시적으로 float로 변환

        # Ackermann 메시지 생성
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = float(velocity)
        cmd.drive.steering_angle = float(steering_angle)
        
        # 명령 발행
        self.tracker_pub.publish(cmd_vel)
        self.erp_pub.publish(cmd)
        
        # 에러 발행
        cte_msg = Float64()
        cte_msg.data = float(self.crosstrack_error)
        self.ct_error_pub.publish(cte_msg)
        
        he_msg = Float64()
        he_msg.data = float(self.heading_error)
        self.h_error_pub.publish(he_msg)

        # 오버레이 텍스트 업데이트
        self.publish_overlay_text()

        # 로그 출력
        self.get_logger().info(f'명령 발행 - 속도: {velocity:.2f} m/s | 조향각: {steering_angle * 180.0 / np.pi:.2f} deg')
        self.get_logger().info(f'오차 - CTE: {self.crosstrack_error:.2f} m | HE: {self.heading_error * 180.0 / np.pi:.2f} deg')

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

    def calculate_curvature(self, points):
        """곡률 계산"""
        curvatures = []
        for i in range(1, len(points)-1):
            # 3점을 이용한 곡률 계산
            p1 = np.array(points[i-1])
            p2 = np.array(points[i])
            p3 = np.array(points[i+1])
            
            # 두 벡터 계산
            v1 = p2 - p1
            v2 = p3 - p2
            
            # 벡터의 길이
            l1 = np.linalg.norm(v1)
            l2 = np.linalg.norm(v2)
            
            # 단위 벡터
            u1 = v1 / l1
            u2 = v2 / l2
            
            # 각도 계산
            angle = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
            
            # 곡률 계산 (각도 / 평균 거리)
            curvature = angle / ((l1 + l2) / 2)
            
            curvatures.append(curvature)
        
        return np.array(curvatures)

    def publish_visualization(self):
        # 곡률 시각화
        curvature_markers = MarkerArray()
        
        # 곡률 마커 생성
        for i, (x, y, curvature) in enumerate(zip(self.cx, self.cy, self.curvatures)):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 위치 설정
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            
            # 크기 설정 (곡률에 비례)
            scale = 0.2 + 0.3 * (curvature / self.max_curvature)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # 색상 설정 (곡률에 비례)
            marker.color.r = min(1.0, curvature / self.max_curvature)
            marker.color.g = 0.0
            marker.color.b = 1.0 - min(1.0, curvature / self.max_curvature)
            marker.color.a = 0.7
            
            curvature_markers.markers.append(marker)
        
        # 참조점 마커 생성
        ref_marker = Marker()
        ref_marker.header.frame_id = "world"
        ref_marker.header.stamp = self.get_clock().now().to_msg()
        ref_marker.id = 0
        ref_marker.type = Marker.ARROW
        ref_marker.action = Marker.ADD
        
        # 현재 참조점 위치
        ref_marker.pose.position.x = self.cx[self.target_ind]
        ref_marker.pose.position.y = self.cy[self.target_ind]
        ref_marker.pose.position.z = 0.0
        
        # 방향 설정
        yaw = self.cyaw[self.target_ind]
        ref_marker.pose.orientation = yaw_to_quaternion(yaw)
        
        # 크기 설정
        ref_marker.scale.x = 1.0  # 화살표 길이
        ref_marker.scale.y = 0.2  # 화살표 너비
        ref_marker.scale.z = 0.2  # 화살표 높이
        
        # 색상 설정
        ref_marker.color.r = 1.0
        ref_marker.color.g = 0.0
        ref_marker.color.b = 0.0
        ref_marker.color.a = 1.0
        
        # 마커 발행
        self.curvature_pub.publish(curvature_markers)
        self.reference_point_pub.publish(ref_marker)

    def calculate_road_boundaries(self, path_points):
        """경로 포인트로부터 도로 경계선 계산"""
        left_boundary = []
        right_boundary = []
        
        for i in range(len(path_points)-1):
            # 현재 점과 다음 점
            p1 = np.array(path_points[i])
            p2 = np.array(path_points[i+1])
            
            # 방향 벡터
            direction = p2 - p1
            direction = direction / np.linalg.norm(direction)
            
            # 수직 벡터 (반시계 방향)
            perpendicular = np.array([-direction[1], direction[0]])
            
            # 경계선 포인트 계산
            left_point = p1 + perpendicular * (self.road_width/2 + self.safety_margin)
            right_point = p1 - perpendicular * (self.road_width/2 + self.safety_margin)
            
            left_boundary.append(left_point)
            right_boundary.append(right_point)
        
        return np.array(left_boundary), np.array(right_boundary)

    def check_boundary_constraints(self, x, y):
        """차량 위치가 도로 경계 내에 있는지 확인"""
        # KDTree를 사용하여 가장 가까운 경계선 포인트 찾기
        left_tree = KDTree(self.left_boundary)
        right_tree = KDTree(self.right_boundary)
        
        # 좌측 경계선까지의 거리
        left_dist, _ = left_tree.query([x, y])
        
        # 우측 경계선까지의 거리
        right_dist, _ = right_tree.query([x, y])
        
        # 제약조건 위반 여부 확인
        left_violation = left_dist < self.safety_margin
        right_violation = right_dist < self.safety_margin
        
        return not (left_violation or right_violation)

    def publish_boundary_visualization(self):
        """도로 경계선 시각화"""
        boundary_markers = MarkerArray()
        
        # 좌측 경계선
        left_marker = Marker()
        left_marker.header.frame_id = "world"
        left_marker.header.stamp = self.get_clock().now().to_msg()
        left_marker.id = 0
        left_marker.type = Marker.LINE_STRIP
        left_marker.action = Marker.ADD
        
        # 좌측 경계선 포인트 추가
        for point in self.left_boundary:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            left_marker.points.append(p)
        
        # 좌측 경계선 스타일 설정
        left_marker.scale.x = 0.1  # 선 두께
        left_marker.color.r = 1.0
        left_marker.color.g = 1.0
        left_marker.color.b = 0.0
        left_marker.color.a = 0.7
        
        # 우측 경계선
        right_marker = Marker()
        right_marker.header.frame_id = "world"
        right_marker.header.stamp = self.get_clock().now().to_msg()
        right_marker.id = 1
        right_marker.type = Marker.LINE_STRIP
        right_marker.action = Marker.ADD
        
        # 우측 경계선 포인트 추가
        for point in self.right_boundary:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            right_marker.points.append(p)
        
        # 우측 경계선 스타일 설정
        right_marker.scale.x = 0.1  # 선 두께
        right_marker.color.r = 1.0
        right_marker.color.g = 1.0
        right_marker.color.b = 0.0
        right_marker.color.a = 0.7
        
        boundary_markers.markers.append(left_marker)
        boundary_markers.markers.append(right_marker)
        
        self.boundary_pub.publish(boundary_markers)

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