#! /usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
from scipy.spatial import KDTree
import math
import time

def rotate_point(x, y, yaw):
    # 회전 변환 행렬 적용
    x_new = x * math.cos(yaw) - y * math.sin(yaw)
    y_new = x * math.sin(yaw) + y * math.cos(yaw)
    return x_new, y_new

def points_on_circle(center, radius, start_angle, end_angle, num_points=10):
    angles = np.linspace(start_angle, end_angle, num_points)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return x, y

def create_straight_path(start, end, step_size):
    x_start, y_start = start
    x_end, y_end = end
    
    # Calculate the total distance
    total_distance = np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)

    if total_distance is None:
        print("start point", x_start, y_start)
        print("end point", x_end, y_end)
    # Calculate the number of points
    num_points = int(total_distance / step_size) + 1
    
    # Generate points along the path
    x_path = np.linspace(x_start, x_end, num_points)
    y_path = np.linspace(y_start, y_end, num_points)
    
    return x_path, y_path

class Delivery():
    def __init__(self, ref_path, car_pose_init, delivery_spot, min_R, delivery_mode):
        self.ref_path = ref_path
        self.car_pose_init = car_pose_init
        self.delivery_spot = delivery_spot[0]
        self.delivery_mode = delivery_mode
        
        # 원의 순서가 진행 순서라고 생각하면 됨(원1 -> 원2 -> 원3 -> 원4)
        # 중간중간 직선구간 포함
        self.min_R = min_R
        # 차의 크기
        self.car_width = 1.16 # 차 폭
        self.car_length = 1.6 # 차 길이

        # 파라미터 조정A
        self.min_R1_A = 7.5 # entry중 시작지점에 가까운 원
        self.min_R2_A = 7.5 # entry중 목표지점에 가까운 원
        self.min_R3_A = 6.0 # exit중 시작지점에 가까운 원
        self.min_R4_A = 6.0 # exit중 목표지점에 가까운 원
        self.spot_adjustment_x_A = 0.3 # 조정위치
        self.spot_adjustment_y_A = 1.2
        self.delivery_margin_A = 2.0 #마진
        
        # 파라미터 조정B
        self.min_R1_B = 2.45 # entry중 시작지점에 가까운 원
        self.min_R2_B = 2.45 # entry중 목표지점에 가까운 원
        self.min_R3_B = 3.2 # exit중 시작지점에 가까운 원s
        self.min_R4_B = 3.2 # exit중 목표지점에 가까운 원
        self.spot_adjustment_x_B = 0.0# 조정위치 
        self.spot_adjustment_y_B = 0.65
        self.delivery_margin_B = 1.9 #마진
        # 조정 위치를 줄이고 마진을 늘린다음에 미리 멈추는걸로 
        self.stop_point = 6 #패스 끝 지점에서 멈추는 거리 설정

        # self.min_R1_B = 2.6 # entry중 시작지점에 가까운 원
        # self.min_R2_B = 2.6 # entry중 목표지점에 가까운 원
        # self.min_R3_B = 4.0 # exit중 시작지점에 가까운 원s
        # self.min_R4_B = 5.0 # exit중 목표지점에 가까운 원
        # self.spot_adjustment_x_B = -1.7# 조정위치 
        # self.spot_adjustment_y_B = 0.5
        # self.delivery_margin_B = 0.8 #마진
        # # 조정 위치를 줄이고 마진을 늘린다음에 미리 멈추는걸로 
        # self.stop_point = 6 #패스 끝 지점에서 멈추는 거리 설정
        # 후진2.0


        
        
        self.path_is_made = False
        self.delivery_status = -1
        # delivery_status 
        # 0: entry
        # 1: exit1
        # 2: exit2
        # 2: ref path 따라가기
        self.delivery_stop_time = None
        self.delivery_onetime_stop = True

        self.spot_adjustment_x, self.spot_adjustment_y = None, None
        
        self.delivery_path_entry_x, self.delivery_path_entry_y = None, None
        self.delivery_path_exit_forward_x, self.delivery_path_exit_forward_y = None, None
        self.delivery_path_exit_backward_x, self.delivery_path_exit_backward_y = None, None

        self.ref_path_kdtree = KDTree(list(zip(ref_path['x'], ref_path['y'])))

        return

    def find_path(self, car_pose, obstacles):
        is_mission_finished = False
        if not self.path_is_made:
            print(self.delivery_spot)
            # 배달 패스 만들기
            self.path_is_made = self.find_delivery_path(car_pose, self.delivery_spot)
            path_x, path_y = self.delivery_path_entry_x, self.delivery_path_entry_y
            self.delivery_status = 0
            gear = 0

        elif self.path_is_made:
            if self.delivery_mode == "delivery_A":
                if self.delivery_status == 0:
                    self.stop_point = 9
                    path_x, path_y = self.delivery_path_entry_x, self.delivery_path_entry_y

                elif self.delivery_status == 1:
                    path_x, path_y = self.delivery_path_exit_forward_x, self.delivery_path_exit_forward_y
                    is_mission_finished = True
                elif self.delivery_status >= 2:
                    
                    path_x, path_y, _ = self.find_ref_path(car_pose,4,15)

            if self.delivery_mode == "delivery_B":
                if self.delivery_status == 0:
                    path_x, path_y = self.delivery_path_entry_x, self.delivery_path_entry_y

                elif self.delivery_status == 1:
                    path_x, path_y = self.delivery_path_exit_backward_x[::-1], self.delivery_path_exit_backward_y[::-1]

                elif self.delivery_status == 2:
                    path_x, path_y = self.delivery_path_exit_forward_x, self.delivery_path_exit_forward_y
                    is_mission_finished = True
                elif self.delivery_status >= 3:
                    path_x, path_y, _ = self.find_ref_path(car_pose,4,15)

            # 현재 path에서 끝부분에 도착하면 다음 status로 변경
            path_kdtree = KDTree(list(zip(path_x, path_y)))
            _, cur_idx = path_kdtree.query(car_pose[:2])
            if self.delivery_onetime_stop == True:
                self.delivery_stop_time = time.time()
                self.delivery_onetime_stop = False
            if cur_idx >= len(path_x)-self.stop_point:
                # if self.delivery_mode == 'delivery_B' and self.delivery_status == 1:
                self.delivery_status += 1
                self.delivery_stop_time = time.time()

            print("delivery_status: ", self.delivery_status)

            gear = self.get_gear_delivery()

        return [], [path_x, path_y, self.calc_path_yaw(path_x, path_y)], False, gear, is_mission_finished
    
    def get_gear_delivery(self):
        if self.delivery_mode == 'delivery_A':
            if self.delivery_status == 0:
                # if time.time() - self.delivery_stop_time <= 2: # neutral
                #     gear = 1
                # else:
                gear = 0

            elif self.delivery_status == 1:
                if time.time() - self.delivery_stop_time <= 5: # neutral
                    gear = 1
                else: gear = 0

            elif self.delivery_status >= 2:
                gear = 0

        if self.delivery_mode =='delivery_B':
            if self.delivery_status == 0:
                # if time.time() - self.delivery_stop_time <= 2: # neutral
                #     gear = 1
                # else:
                self.stop_point = 11
                gear = 0

            elif self.delivery_status == 1:
                self.stop_point = 1
                if time.time() - self.delivery_stop_time <= 5: # neutral
                    gear = 1
                else:
                    gear = 2 #후진

            elif self.delivery_status == 2:
                self.stop_point = 6
                if time.time() - self.delivery_stop_time <= 1: # neutral
                    gear = 1
                else: gear = 0 #전진

            elif self.delivery_status >= 3:
                gear = 0
            
        return gear

    def find_delivery_path(self, car_pose, delivery_spot_utm):
        delivery_spot = self.utm_2_bl(car_pose, delivery_spot_utm)
        print("내 현재 위치:",car_pose)
        print("배달 위치(velodyne_Frame)",delivery_spot)
        print("delivery_mode", self.delivery_mode)
    
        # 1. delivery spot을 회전시키기(ref_path의 yaw만큼)
        # -> bl의 프레임의 경우 나의 yaw값에 영향을 받기 때문에 path의 계산의 편의성을 위해서 회전시킴
        # base_link에서 ref_path의 yaw값
        ref_path_x, ref_path_y,idx = self.find_ref_path(car_pose, 3, 4) # 앞의 점 1개만 했는데 잘 안나오면 늘려야함(이부분 좀 깔끔하게 변경했으면 좋겠음)
        ref_path_x_bl, ref_path_y_bl = self.transform_waypoints_utm_2_bl(car_pose, ref_path_x, ref_path_y)
        delivery_yaw = self.calc_path_yaw(ref_path_x_bl, ref_path_y_bl)
        delivery_spot[0], delivery_spot[1] = rotate_point(delivery_spot[0], delivery_spot[1], -delivery_yaw[idx])
        
        if self.delivery_mode == "delivery_A":
            delivery_spot[0] += self.spot_adjustment_x_A
            delivery_spot[1] += self.spot_adjustment_y_A

            # 2. 회전시킨 delivery spot에서 1.0m 아래으로 이동 매끄러운 진입을 위해 목표지점을 내림
            delivery_spot[0] -= self.delivery_margin_A #아래
            
            # 3. 원2의 중심과 반지름
            radius2 = self.min_R2_A # 최소회전반경
            center2 = (delivery_spot[0],delivery_spot[1] + radius2)
            
            # 4. 원1의 중심과 반지름
            radius1 = self.min_R1_A
            y1 = 0 - radius1
            distance = radius1 + radius2 
            delta_y = center2[1] - y1
            print(distance,delta_y)
            delta_x = np.sqrt(distance**2 - delta_y**2)
            x1 = delivery_spot[0] - delta_x
            center1 = (x1, y1)

            # 5. 원3의 중심과 반지름(배달 구역에서 나가기 위한 원) -> 만약 배달 구역이 좀 좁다면 min_R을 줄여야함
            radius3 = self.min_R3_A
            center3 = (delivery_spot[0] + self.delivery_margin_A ,delivery_spot[1] + radius3)

            # 6. 원4의 중심과 반지름
            radius4 = self.min_R4_A
            y4 = 0 - radius4
            distance = radius3 + radius4
            delta_y = center3[1] - y4
            delta_x = np.sqrt(distance**2 - delta_y**2)      
            x4 = delivery_spot[0] + self.delivery_margin_A + delta_x 
            center4 = (x4, y4)

            p0 = (-1.0,0)
            p1 = (center1[0],0)
            print(p0,p1)
            p2 = ((center1[0]+center2[0])/2, (center1[1]+center2[1])/2)
            p3 = (delivery_spot[0], delivery_spot[1])
            p4 = (delivery_spot[0] + self.delivery_margin_A, delivery_spot[1])
            p5 = ((center3[0]+center4[0])/2, (center3[1]+center4[1])/2)
            p6 = (center4[0],0)

            # 진입경로
            path_entry_x = []
            path_entry_y = []

            step_size = 0.2
            # path1: 회전하기 전까지의 직진 경로
            rotate_path1_x, rotate_path1_y = create_straight_path(p0, p1, step_size)

            for x, y in zip(rotate_path1_x, rotate_path1_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_entry_x.append(x_new)
                path_entry_y.append(y_new)
                
            # path2: 회전 시작후 목표지점까지의 경로
            # 시작점에서 첫 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle1 = np.arctan2(p1[1] - center1[1], p1[0] - center1[0])
            end_angle1 = np.arctan2(p2[1] - center1[1], p2[0] - center1[0])
            p_x1, p_y1 = points_on_circle(center1, radius1, start_angle1, end_angle1)

            # 두 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle2 = np.arctan2(p2[1] - center2[1], p2[0] - center2[0])
            end_angle2 = np.arctan2(p3[1] - center2[1], p3[0] - center2[0])
            p_x2, p_y2 = points_on_circle(center2, radius2, start_angle2, end_angle2)

            rotate_path2_x = np.concatenate([p_x1, p_x2])
            rotate_path2_y = np.concatenate([p_y1, p_y2])

            for x, y in zip(rotate_path2_x, rotate_path2_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_entry_x.append(x_new)
                path_entry_y.append(y_new)

            # path3: 차체의 똑바른 정렬을 위한 직진 경로 
            rotate_path3_x, rotate_path3_y = create_straight_path(p3, p4, step_size)

            for x, y in zip(rotate_path3_x, rotate_path3_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_entry_x.append(x_new)
                path_entry_y.append(y_new)

            # 이제 나갈 차례
            path_exit_x = []
            path_exit_y = []

            start_angle1 = np.arctan2(p4[1] - center3[1], p4[0] - center3[0])
            end_angle1 = np.arctan2(p5[1] - center3[1], p5[0] - center3[0])
            p_x3, p_y3 = points_on_circle(center3, radius3, start_angle1, end_angle1)

            # 두 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle2 = np.arctan2(p5[1] - center4[1], p5[0] - center4[0])
            end_angle2 = np.arctan2(p6[1] - center4[1], p6[0] - center4[0])
            p_x4, p_y4 = points_on_circle(center4, radius4, start_angle2, end_angle2)

            rotate_path4_x = np.concatenate([p_x3, p_x4])
            rotate_path4_y = np.concatenate([p_y3, p_y4])

            for x, y in zip(rotate_path4_x, rotate_path4_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_exit_x.append(x_new)
                path_exit_y.append(y_new)
            
            self.delivery_path_entry_x, self.delivery_path_entry_y = self.transform_waypoints_bl_2_utm(car_pose, path_entry_x, path_entry_y)
            self.delivery_path_exit_forward_x, self.delivery_path_exit_forward_y = self.transform_waypoints_bl_2_utm(car_pose, path_exit_x, path_exit_y)
        
        elif self.delivery_mode == "delivery_B":
            delivery_spot[0] += self.spot_adjustment_x_B
            delivery_spot[1] += self.spot_adjustment_y_B

            # 2. 회전시킨 delivery spot에서 1.0m 아래으로 이동 매끄러운 진입을 위해 목표지점을 내림
            delivery_spot[0] -= self.delivery_margin_B #아래
            
            # 3. 원2의 중심과 반지름
            radius2 = self.min_R2_B
            center2 = (delivery_spot[0],delivery_spot[1] + radius2)
            
            # 4. 원1의 중심과 반지름
            radius1 = self.min_R1_B
            y1 = 0 - radius1
            distance = radius1 + radius2
            delta_y = center2[1] - y1
            delta_x = np.sqrt(distance**2 - delta_y**2)
            x1 = delivery_spot[0] - delta_x
            center1 = (x1, y1)

            # 후진 위치 잡기
            # 나가기 전에 후진의 위치를 잡아야함
            # self.car_width = 1.16
            # self.car_length = 1.6 
            Front_Right_R = np.sqrt((self.min_R3_B + self.car_width/2)**2 + (self.car_length/2)**2) 
            # 표지판으로부터 떨어져야하는 거리
            back_distance = np.sqrt(Front_Right_R**2-self.min_R3_B**2)
            back_distance_v1 = back_distance - self.spot_adjustment_x_B # 내 현재 위치로부터 떨어져야하는 거리(대략 2미터 나오려나)
            back_distance_v1 = back_distance_v1 + 0.4

            # 5. 원3의 중심과 반지름(배달 구역에서 나가기 위한 원)
            radius3 = self.min_R3_B
            center3 = (delivery_spot[0] + self.delivery_margin_B - back_distance_v1, delivery_spot[1] + radius3)

            # 6. 원4의 중심과 반지름 
            radius4 = self.min_R4_B
            y4 = 0 - radius4
            distance = radius3 + radius4
            delta_y = center3[1] - y4
            delta_x = np.sqrt(distance**2 - delta_y**2)     
            x4 = delivery_spot[0] + self.delivery_margin_B - back_distance_v1 + delta_x
            center4 = (x4, y4)

            # 후진 위치 잡기
            p0 = (-1.0,0)
            p1 = (center1[0],0)
            p2 = ((center1[0]+center2[0])/2, (center1[1]+center2[1])/2)
            p3 = (delivery_spot[0], delivery_spot[1]) #들어가는 위치 & 나가는 위치
            p4 = (delivery_spot[0] + self.delivery_margin_B, delivery_spot[1]) # 이 점이 가장 최종 주차 위치
            p3_b = (delivery_spot[0] + self.delivery_margin_B - back_distance_v1 , delivery_spot[1]) #후진 위치
            p5 = ((center3[0]+center4[0])/2, (center3[1]+center4[1])/2)
            p6 = (center4[0],0)

            # 진입경로
            # p0 -> p1 -> p2 -> p3 -> p4
            path_entry_x = []
            path_entry_y = []

            step_size = 0.2
            # path1: 회전하기 전까지의 직진 경로
            rotate_path1_x, rotate_path1_y = create_straight_path(p0, p1, step_size)

            for x, y in zip(rotate_path1_x, rotate_path1_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_entry_x.append(x_new)
                path_entry_y.append(y_new)
                
            # path2: 회전 시작후 목표지점까지의 경로
            # 시작점에서 첫 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle1 = np.arctan2(p1[1] - center1[1], p1[0] - center1[0])
            end_angle1 = np.arctan2(p2[1] - center1[1], p2[0] - center1[0])
            p_x1, p_y1 = points_on_circle(center1, radius1, start_angle1, end_angle1)

            # 두 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle2 = np.arctan2(p2[1] - center2[1], p2[0] - center2[0])
            end_angle2 = np.arctan2(p3[1] - center2[1], p3[0] - center2[0])
            p_x2, p_y2 = points_on_circle(center2, radius2, start_angle2, end_angle2)

            rotate_path2_x = np.concatenate([p_x1, p_x2])
            rotate_path2_y = np.concatenate([p_y1, p_y2])

            for x, y in zip(rotate_path2_x, rotate_path2_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_entry_x.append(x_new)
                path_entry_y.append(y_new)

            # path3: 차체의 똑바른 정렬을 위한 직진 경로 
            rotate_path3_x, rotate_path3_y = create_straight_path(p3, p4, step_size)

            for x, y in zip(rotate_path3_x, rotate_path3_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_entry_x.append(x_new)
                path_entry_y.append(y_new)

            # 이제 나갈 차례
            # (후진)p4 -> p3 -> (전진)p5 -> p6

            # 후진
            path_exit_backward_x = []
            path_exit_backward_y = []
            
            rotate_path4_x, rotate_path4_y = create_straight_path(p3_b, p4, step_size)

            for x, y in zip(rotate_path4_x, rotate_path4_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])        
                path_exit_backward_x.append(x_new)
                path_exit_backward_y.append(y_new)

            # 전진
            path_exit_forward_x = []
            path_exit_forward_y = []

            start_angle1 = np.arctan2(p3_b[1] - center3[1], p3_b[0] - center3[0])
            end_angle1 = np.arctan2(p5[1] - center3[1], p5[0] - center3[0])
            p_x3, p_y3 = points_on_circle(center3, radius3, start_angle1, end_angle1)

            # 두 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle2 = np.arctan2(p5[1] - center4[1], p5[0] - center4[0])
            end_angle2 = np.arctan2(p6[1] - center4[1], p6[0] - center4[0])
            p_x4, p_y4 = points_on_circle(center4, radius4, start_angle2, end_angle2)

            rotate_path5_x = np.concatenate([p_x3, p_x4])
            rotate_path5_y = np.concatenate([p_y3, p_y4])

            for x, y in zip(rotate_path5_x, rotate_path5_y):
                x_new, y_new = rotate_point(x, y, delivery_yaw[idx])
                path_exit_forward_x.append(x_new)
                path_exit_forward_y.append(y_new)
            
            self.delivery_path_entry_x, self.delivery_path_entry_y = self.transform_waypoints_bl_2_utm(car_pose, path_entry_x, path_entry_y)
            self.delivery_path_exit_backward_x, self.delivery_path_exit_backward_y = self.transform_waypoints_bl_2_utm(car_pose, path_exit_backward_x, path_exit_backward_y)
            self.delivery_path_exit_forward_x, self.delivery_path_exit_forward_y = self.transform_waypoints_bl_2_utm(car_pose, path_exit_forward_x, path_exit_forward_y)
    
        return True
    
    def find_ref_path(self, car_pose, n_back, n_forward):
        _, idx = self.ref_path_kdtree.query(car_pose[:2])

        # n_back = 4
        # n_forward = 15
        start_index = max(idx - n_back, 0)
        end_index = min(idx + n_forward, len(self.ref_path['x']))
        relative_idx = idx - start_index #path에서의 나의 위치
        # 인근 waypoints 추출
        path_x = self.ref_path['x'][start_index:end_index + 1]
        path_y = self.ref_path['y'][start_index:end_index + 1]
        return path_x, path_y, relative_idx
    
    @ staticmethod
    def calc_path_yaw(path_x, path_y):
        path_yaw = []
        for i in range(len(path_x) - 2):
            # Calculate the differences in x and y
            dx = path_x[i + 1] - path_x[i]
            dy = path_y[i + 1] - path_y[i]

            # Calculate the yaw angle using atan2, which handles all quadrants
            yaw = math.atan2(dy, dx)

            # Append the calculated yaw to the list
            path_yaw.append(yaw)
        
        path_yaw.append(path_yaw[-1])
        return path_yaw
    
    @staticmethod
    def transform_waypoints_bl_2_utm(car_pose, path_x_bl, path_y_bl):
        path_x_global = []
        path_y_global = []

        # Convert each point in the path to the global frame
        for x_bl, y_bl in zip(path_x_bl, path_y_bl):
            # Rotate the points
            x_rot = x_bl * math.cos(car_pose[2]) - y_bl * math.sin(car_pose[2])
            y_rot = x_bl * math.sin(car_pose[2]) + y_bl * math.cos(car_pose[2])

            # Translate the points
            x_global = x_rot + car_pose[0]
            y_global = y_rot + car_pose[1]

            # Append the transformed coordinates to the lists
            path_x_global.append(x_global)
            path_y_global.append(y_global)

        return path_x_global, path_y_global
    
    @staticmethod
    def transform_waypoints_utm_2_bl(car_pose, path_x_utm, path_y_utm):
        path_x_bl = []
        path_y_bl = []
        # Convert each point in the path to the global frame
        for x_utm, y_utm in zip(path_x_utm, path_y_utm):
            translated_x = x_utm - car_pose[0]
            translated_y = y_utm - car_pose[1]
            # Rotate the coordinates to align with the car's orientation
            base_link_x = translated_x * math.cos(-car_pose[2]) - translated_y * math.sin(-car_pose[2])
            base_link_y = translated_x * math.sin(-car_pose[2]) + translated_y * math.cos(-car_pose[2])
            # Append the transformed coordinates to the lists
            path_x_bl.append(base_link_x)
            path_y_bl.append(base_link_y)
        return path_x_bl, path_y_bl
    
    @ staticmethod
    def calc_path_yaw(path_x, path_y):
        path_yaw = []
        for i in range(len(path_x) - 2):
            # Calculate the differences in x and y
            dx = path_x[i + 1] - path_x[i]
            dy = path_y[i + 1] - path_y[i]

            # Calculate the yaw angle using atan2, which handles all quadrants
            yaw = math.atan2(dy, dx)

            # Append the calculated yaw to the list
            path_yaw.append(yaw)
        
        path_yaw.append(path_yaw[-1])
        return path_yaw

    def bl_2_utm(self, car_pose, point):
        x_rot = point[0] * math.cos(car_pose[2]) - point[1] * math.sin(car_pose[2])
        y_rot = point[0] * math.sin(car_pose[2]) + point[1] * math.cos(car_pose[2])

        # Translate the points
        x_global = x_rot + car_pose[0]
        y_global = y_rot + car_pose[1]
        return (x_global, y_global)
    
    def utm_2_bl(self, car_pose, point):
        # Translate the obstacle coordinates
        translated_x = point[0] - car_pose[0]
        translated_y = point[1] - car_pose[1]

        # Rotate the coordinates to align with the car's orientation
        base_link_x = translated_x * math.cos(-car_pose[2]) - translated_y * math.sin(-car_pose[2])
        base_link_y = translated_x * math.sin(-car_pose[2]) + translated_y * math.cos(-car_pose[2])
        return [base_link_x, base_link_y]
    
    @staticmethod
    def transform_obstacles_utm_2_bl(car_pose, obstacles):
        transformed_obstacles = []

        for obstacle in obstacles:
            obs_x, obs_y, r = obstacle

            # Translate the obstacle coordinates
            translated_x = obs_x - car_pose[0]
            translated_y = obs_y - car_pose[1]

            # Rotate the coordinates to align with the car's orientation
            base_link_x = translated_x * math.cos(-car_pose[2]) - translated_y * math.sin(-car_pose[2])
            base_link_y = translated_x * math.sin(-car_pose[2]) + translated_y * math.cos(-car_pose[2])

            # Keep the radius the same
            transformed_obstacles.append([base_link_x, base_link_y, r])

        return transformed_obstacles




