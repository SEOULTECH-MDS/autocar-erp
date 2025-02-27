#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node 
import pyproj
import numpy as np
import time
import copy
from scipy.spatial import KDTree

from collections import deque

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped, PoseArray,TwistWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int16, Int32, Float64MultiArray, String, Bool
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32

# from carla_msgs.msg import CarlaEgoVehicleControl

from autocar_nav.euler_from_quaternion import euler_from_quaternion
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion

SPEED = 2.5

class LowPassFilter():
    def __init__(self, alpha=0.3):
        self.v_prev = 0.0
        self.alpha = alpha
        return
    
    def update(self, v):
        self.v_prev = v * self.alpha + self.v_prev * (1-self.alpha)
        return self.v_prev


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.way_time = None
        self.way_signal = False

        self.gps_pose = Odometry()
        self.gps_pose.header.frame_id = 'world'

        self.location_corrected = Odometry()
        self.location_corrected.header.frame_id = 'world'
        self.location_corrected.pose.pose.orientation.x, self.location_corrected.pose.pose.orientation.y, \
        self.location_corrected.pose.pose.orientation.z, self.location_corrected.pose.pose.orientation.w = 0.0,0.0,0.0,1.0
        
        self.location_long_corrected = Odometry()
        self.location_long_corrected.header.frame_id = 'world'
        
        self.location_dr = Odometry()
        self.location_dr.header.frame_id = 'world'
        
        self.location = Odometry()
        self.location.header.frame_id = 'world'
        
        self.yaw_offset = 0.0 # radians
        self.global_yaw = 0.0
        
        #횡방향 관련 초기값
        self.local_cte = None
        self.lateral_offset = (0,0) # meters
        self.lateral_error = 0.0
        
        #종방향 관련 초기값
        self.stopline_dict = {}
        self.local_ate = 0.0
        self.longitudinal_error = 0.0
        self.longitudinal_offset = (0,0)
        self.is_longitudinal_error_calculated = True
        self.closest_stopline_prev = 0.0
        
        # 방향정보 관련 초기값
        self.location_history = deque(maxlen=5)
        self.gps_mean_yaw = None
        
        # waypoint 초기값
        self.closest_waypoints = []
        self.global_waypoints = []
   
        self.driving_mode = None
        
        #DeadReckoning 초기값
        self.init_pose_set = False
        self.dt = 0.1
        theta = np.radians(-1.4440643432812905)
        self.RM_offset = np.array([
                                   [np.cos(theta), np.sin(-theta), 0, 0],
                                   [np.sin(theta), np.cos(theta) , 0, 0],
                                   [0,             0,              1, 0],
                                   [0,             0,              0, 1]
                                ])
        
        self.init_pose = np.zeros((4, 1))
        self.car_pose_dr = np.zeros((4, 1))
        self.car_pose_dr_offset = np.zeros((4, 1))
        self.steer = 0.0
        self.lpf = LowPassFilter(alpha=0.3)
        self.speed = 0.0
        self.yaw_gps_offset =0.0
        # DeadReckoning 파라미터 
        wheelbase = 1.566 # [m] # 휠베이스 조정
        self.l_r = wheelbase * 0.0 # [m]  #rear
        self.l_f = wheelbase * 1.0 # [m]  #front 무게중심 비율 조정
        
        self.is_go = True

        self.vehicle_length = 2.0
        self.vehicle_width = 1.0

        # ====================== subscription ======================
        # way change signal sub
        self.way_change_signal_sub = self.create_subscription(Bool, '/way_change_signal', self.callback_way_change_signal, 10)

        # gps, imu sub
        self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps/fix',  self.callback_gps, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.callback_imu, 10)
        
        # 엔코더, gps 속도
        self.speed_sub = self.create_subscription(Float64,  "/cur_speed", self.callback_speed, 10)
        # self.speed_sub = self.create_subscription(TwistWithCovarianceStamped, "/ublox_gps/fix_velocity", self.callback_speed, 10)
        
        #erp cmd
        self.cmd_sub = self.create_subscription(AckermannDrive, '/erp/cmd_vel', self.callback_cmd, 10)

        # 현재 주행 모드 가져오기
        self.driving_mode_sub = self.create_subscription(String, '/driving_mode',  self.callback_driving_mode, 10)
        
        #초기 yaw 설정
        #self.init_orientation_sub = self.create_subscription(PoseWithCovarianceStamped, '/initial_global_pose', self.callback_init_orientation, 10)
        self.init_orientation_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback_init_orientation, 10)
        
        # global path planning waypoint 정보
        self.closest_waypoints_sub = self.create_subscription(PoseArray, '/global_closest_waypoints', self.callback_closest_waypoints, 10)
        self.waypoints_sub = self.create_subscription(PoseArray, '/global_waypoints',  self.callback_global_waypoints, 10)
        
        # 정지선 위치
        self.stoplines_sub = self.create_subscription(MarkerArray, '/stoplines', self.callback_stopline, 10)
        
        #global local 횡방향 종방향 에러 sub
        self.local_cte_sub = self.create_subscription(Float64, '/autocar/cte', self.callback_cte, 10)
        self.local_ate_sub = self.create_subscription(Float64, '/autocar/he', self.callback_ate, 10)
        
        # CARLA
        # self.gps_sub = self.create_subscription(NavSatFix, '/carla/ego_vehicle/gnss', self.callback_gps, 10)
        # self.imu_sub = self.create_subscription(Imu, '/carla/ego_vehicle/imu', self.callback_imu, 10)
        # self.speed_sub = self.create_subscription(Float64, '/carla/ego_vehicle/speedometer', self.callback_speed), 10
        # self.cmd_sub = self.cmd_sub = self.create_subscription(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', self.callback_cmd, 10)

        # ====================== publisher ======================
        # 보정안된 위치와 보정된 위치 pub
        self.location_no_correction_pub = self.create_publisher(Odometry, '/autocar/location_not_corrected', 10) # 보정되지 않은 위치
        # self.location_long_corrected_pub = self.create_publisher(Odometry, '/location_long_corrected', 10)
        # self.location_corrected_pub = self.create_publisher(Odometry, '/location_corrected', 10)
        # self.location_dr_pub = self.create_publisher(Odometry, '/location_dr', 10)
        self.location_pub = self.create_publisher(Odometry, '/autocar/location', 10) # 보정된 위치

        # rviz 시각화
        self.location_viz_pub = self.create_publisher(PolygonStamped, '/autocar/viz_location', 10)
        
        # 최종 보정된 종방향 횡방향 에러 pub
        self.lateral_error_pub = self.create_publisher(Float64, '/lateral_error', 10)
        self.longitudinal_error_pub = self.create_publisher(Float64, '/longitudinal_error', 10)

        # 0.1초마다 callback함수 계산해주기
        self.timer_location_publish = self.create_timer(0.1, self.callback_timer_location_pub)
    
    
    def callback_timer_location_pub(self):
     
        location_gps = (self.gps_pose.pose.pose.position.x, self.gps_pose.pose.pose.position.y)
        
        if self.global_waypoints and self.driving_mode == 'normal_driving':
            global_cte = get_cte(location_gps,self.global_waypoints)
        else:
            pass

        if self.closest_waypoints:
            global_ate, closest_stopline = get_ate(location_gps,self.stopline_dict,self.closest_waypoints)
            
            if closest_stopline != self.closest_stopline_prev:
                self.is_longitudinal_error_calculated = True
            self.closest_stopline_prev = closest_stopline
            
            if 3.1 < self.local_ate < 7.5 and 0 < global_ate < 25 and self.is_longitudinal_error_calculated:  #base_link에서 차 위치가 (2,0) 으로 시작하기 때문에
                self.longitudinal_error = self.local_ate - global_ate
            
                parallel_direction = self.global_yaw
                perpendicular_direction = self.global_yaw + np.pi/2
                #print(parallel_direction)
                self.longitudinal_offset = (self.longitudinal_error * np.cos(parallel_direction)+self.lateral_error*np.cos(perpendicular_direction),
                                            self.longitudinal_error * np.sin(parallel_direction)+self.lateral_error*np.sin(perpendicular_direction))
                
                self.is_longitudinal_error_calculated = False
 
            # print(closest_stopline,self.closest_stopline_prev,self.is_longitudinal_error_calculated)
        
        if self.driving_mode == "normal_driving":
            if  (self.way_time is not None) and (self.way_signal == True) and (1.5 < time.time() - self.way_time < 4):
                #횡방향 위치 보정
                local_cte = self.local_cte
                if local_cte is not None:
                    lateral_error = local_cte - global_cte
                    self.lateral_error = lateral_error
                            
                    perpendicular_direction = self.global_yaw + np.pi/2
                    self.lateral_offset = (self.lateral_error * np.cos(perpendicular_direction), self.lateral_error * np.sin(perpendicular_direction))
                    print("보정됨")
                    self.way_signal = False

            #종방향 위치 보정
        
        if self.location is None:
            self.location = copy.deepcopy(self.gps_pose)

        self.location_history.append((self.gps_pose.pose.pose.position.x, self.gps_pose.pose.pose.position.y))
        location_xs = np.array([point[0] for point in self.location_history])
        location_ys = np.array([point[1] for point in self.location_history])
            
        if len(self.location_history) == 5:
            dxs = np.diff(location_xs)
            dys = np.diff(location_ys)
            slopes_np = np.arctan2(dys , dxs)
            
            self.gps_mean_yaw = np.mean(slopes_np)
            

        if self.speed > SPEED and self.is_go == True:
            self.yaw_gps_offset = normalize_angle(self.global_yaw - self.gps_mean_yaw)
            self.location_history.clear()
            self.is_go = False

            # print("IMU 보정함")
        
        elif self.speed > SPEED and np.abs(self.gps_mean_yaw - self.yaw_corr) > np.pi/6:
            self.is_go = True
        else:
            pass

        self.yaw_corr = self.global_yaw - self.yaw_gps_offset
        q = yaw_to_quaternion(self.global_yaw)
        self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y, \
        self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w = q.x, q.y, q.z, q.w
        # if self.speed > 2.5:
        #     self.is_go = True  
        #     self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y, \
        #     self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w = yaw_to_quaternion(self.gps_mean_yaw)
        #     print("gps방향")
        # else :
        #     self.is_go = False
        #     self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y, \
        #     self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w = yaw_to_quaternion(self.global_yaw-self.yaw_gps_offset)
        #     print("imu방향")

        # if self.is_go and (self.gps_mean_yaw is not None) and (np.abs(self.gps_mean_yaw - self.global_yaw) < np.pi/2):
        #     self.yaw_gps_offset = normalize_angle(self.global_yaw - self.gps_mean_yaw) 
        

        self.location.pose.pose.position.x = self.gps_pose.pose.pose.position.x - self.lateral_offset[0] - self.longitudinal_offset[0]
        self.location.pose.pose.position.y = self.gps_pose.pose.pose.position.y - self.lateral_offset[1] - self.longitudinal_offset[1]
        self.location.pose.pose.orientation = self.gps_pose.pose.pose.orientation
        # self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y, \
        # self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w = yaw_to_quaternion(self.gps_mean_yaw)        
        
        
        # Publish
        self.location_no_correction_pub.publish(self.gps_pose)
        # self.location_long_corrected_pub.publish(self.location_long_corrected)
        # self.location_corrected_pub.publish(self.location_corrected)
        # self.location_dr_pub.publish(self.location_dr)
        self.location_pub.publish(self.location) 
        self.corrected_yaw = euler_from_quaternion(self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y, \
                                           self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w)
        
        self.get_logger().info(f"Corrected Global location: x={self.location.pose.pose.position.x}, \
                               y={self.location.pose.pose.position.y}, \
                                yaw={self.corrected_yaw}")
        
        # rviz 차량 위치 시각화
        corners = self.get_vehicle_corners(self.corrected_yaw)

        # Polygon 메시지 생성 및 퍼블리시
        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.header.frame_id = "base_link"   # 차량의 base_link 좌표 중심으로 시각화
        for corner in corners:
            point = Point32()
            point.x, point.y = corner
            polygon_msg.polygon.points.append(point)
        self.location_viz_pub.publish(polygon_msg)

        lateral_error_msg = Float64()
        lateral_error_msg.data = self.lateral_error
        self.lateral_error_pub.publish(lateral_error_msg)
        
        longitudinal_error_msg = Float64()
        longitudinal_error_msg.data = self.longitudinal_error
        self.longitudinal_error_pub.publish(longitudinal_error_msg)

    def get_vehicle_corners(self, yaw):
        """ 차량 중심과 yaw를 기준으로 사각형 네 꼭짓점 좌표 계산 """
        half_length = self.vehicle_length / 2
        half_width = self.vehicle_width / 2

        # 차량 좌표 기준 네 개 꼭짓점 (로컬 좌표)
        corners_local = np.array([
            [ half_length,  half_width],  # front-right
            [ half_length, -half_width],  # front-left
            [-half_length, -half_width],  # rear-left
            [-half_length,  half_width]   # rear-right
        ])

        # 회전 변환 (yaw 적용)
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])
        rotated_corners = (rotation_matrix @ corners_local.T).T

        # 전역 좌표 변환 (차량 중심 좌표를 더함)
        # global_corners = rotated_corners + np.array([x, y])

        return rotated_corners


    def motion_model(self, x, u):
        # x = [x,y,yaw,v].T
        # u = [yaw+beta, v]
        F = np.array([[1.0, 0, 0, 0],
                        [0, 1.0, 0, 0],
                        [0, 0, 1.0, 0],
                        [0, 0, 0, 0.0]])

        B = np.array([[0.0, self.dt * np.cos(u[0,0])],
                        [0.0, self.dt * np.sin(u[0,0])],
                        [0.0, 0.0],
                        [0.0, 1.0]])

        x = F @ x + B @ u
        return x
        
    def callback_driving_mode(self, mode_msg):
        self.driving_mode = mode_msg.data

        # if self.driving_mode != 'intersect': # and self.driving_mode != 'obstacle_avoiding' :
        #     self.init_pose_set = False
            
    
    def callback_way_change_signal(self, signal_msg):
        # way바뀌면 callback 작동
        if self.way_signal == False:
            self.way_time = time.time()
            self.way_signal = True

        return
    
    def callback_gps(self, gps_msg):
        self.gps_pose.pose.pose.position.x, self.gps_pose.pose.pose.position.y = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)
        
        
    def callback_imu(self,imu_msg):
        local_yaw = euler_from_quaternion(imu_msg.orientation.x, imu_msg.orientation.y,\
                                          imu_msg.orientation.z, imu_msg.orientation.w)
        global_yaw = local_yaw + self.yaw_offset
        self.global_yaw = normalize_angle(global_yaw)
        self.car_pose_dr[2, 0] = self.global_yaw
        
    def callback_cmd(self, cmd_msg):
        steer = -np.radians(cmd_msg.steering_angle)
        #erp
        if steer > 20:
            steer = 20
        elif steer < -20:
            steer = -20
        else:
            steer = steer

        self.steer = self.lpf.update(steer)
        #CARLA
        # self.steer = self.lpf.update(-cmd_msg.steer)
    
    def callback_speed(self, speed_msg):
        # speed = np.sqrt(speed_msg.twist.twist.linear.x **2 + speed_msg.twist.twist.linear.y**2)
        self.speed = speed_msg.data
        self.car_pose_dr[3, 0] = self.speed
        
        
    def callback_init_orientation(self, init_pose_msg):
        global_yaw = euler_from_quaternion(init_pose_msg.pose.pose.orientation.x, init_pose_msg.pose.pose.orientation.y, \
                                           init_pose_msg.pose.pose.orientation.z, init_pose_msg.pose.pose.orientation.w)
       
        local_yaw = euler_from_quaternion(self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y,\
                                          self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w)

        self.yaw_offset += global_yaw - local_yaw
    
    def callback_closest_waypoints(self, poses_msg):
        closest_waypoints = []
        for pose in poses_msg.poses:
            x = pose.position.x
            y = pose.position.y
            closest_waypoints.append((x, y))
        self.closest_waypoints = closest_waypoints
        
    def callback_global_waypoints(self,poses_msg):
        global_waypoint = []
        for pose in poses_msg.poses:
            x = pose.position.x
            y = pose.position.y
            global_waypoint.append((x, y))
        self.global_waypoints = global_waypoint
        
        
    def callback_cte(self, local_cte_msg):
        self.local_cte = local_cte_msg.data
        
    
    def callback_ate(self, local_ate_msg):
        self.local_ate = local_ate_msg.data
    
    def callback_stopline(self,stoplines_msg):
        for stopline in stoplines_msg.markers:
            # marker의 id를 추출
            marker_id = stopline.id
            c1 = (stopline.points[0].x,stopline.points[0].y)
            c2 = (stopline.points[1].x,stopline.points[1].y)
                        
            # id를 키로 하고 좌표 (x, y)를 배열로 저장
            if marker_id not in self.stopline_dict:
                self.stopline_dict[marker_id] = []

            # x, y 좌표를 리스트에 추가
            self.stopline_dict[marker_id].append([c1,c2])
        
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def normalize_angle(angle):
    
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
        
    return angle

def get_cte(car_position, waypoints):
        waypoints_kdtree = KDTree(waypoints)
        _, closest_node_idx = waypoints_kdtree.query(car_position)
        try:
            first_node = waypoints[closest_node_idx]
            second_node = waypoints[closest_node_idx + 1]
        except IndexError:
            first_node = waypoints[closest_node_idx - 1]
            second_node = waypoints[closest_node_idx]
            print('err')
        linear_vector = [(second_node[0] - first_node[0]) , (second_node[1] - first_node[1])]
        slope = linear_vector[1]/linear_vector[0]
                
        intercept = first_node[1] - slope*first_node[0]
        # print(linear_vector,slope,intercept)
        if linear_vector[0] >= 0:
            line_coef = [slope, -1, intercept] # ax-y+b=0
        # elif waypoint 방향 감소: -ax +y -b=0
        else:
            line_coef = [-slope, 1, -intercept] # ax+y+b=0
            
        cross_track_error = (line_coef[0]*car_position[0] + line_coef[1]*car_position[1] + line_coef[2])\
                                / np.sqrt(line_coef[0]**2 + line_coef[1]**2)
        # print(cross_track_error)
        return cross_track_error

def get_ate(car_position, stopline, near_waypoints):
        closest_stopline = None
        closest_coordinates = None
        min_distance = np.inf

        near_waypoints_kdtree = KDTree(near_waypoints)

        for stopline_id, stopline_node in stopline.items():
            coordinates = stopline_node[0]
            stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*coordinates)]
            
            # 인접 waypoints가 정지선을 포함하는지 확인
            per_distance, _ = near_waypoints_kdtree.query(stp_mid_point)
            
            if per_distance > 2: 
                continue

            distance = (car_position[0] - stp_mid_point[0])**2 + (car_position[1] - stp_mid_point[1])**2
            if distance < min_distance:
                min_distance = distance
                closest_stopline = stopline_id
                closest_coordinates = coordinates

        along_track_error = 0

        if closest_stopline is None:
            along_track_error = 1000000
            return along_track_error, closest_stopline
        
        point1 = closest_coordinates[0]
        point2 = closest_coordinates[1]

        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        rad = np.arctan2(dy, dx) # atan2는 두 수의 비율에 대한 아크탄젠트를 반환
        
        m = dy/dx # 기울기
        b = point1[1] - m * point1[0] # y절편

        # 점과 직선의 거리
        along_track_error = abs(m * car_position[0] - car_position[1] + b) / np.sqrt(m ** 2 + 1)

        return along_track_error, closest_stopline

def main(args=None):
    
    # Initialise the node
    rclpy.init(args=args)
    
    try:
        # Initialise the class
        localization = Localization()

        # Stop the node from exiting
        rclpy.spin(localization)

    finally:
        localization.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()