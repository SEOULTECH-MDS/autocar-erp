#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree

from autocar_nav.osm_handler import OSMHandler
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from autocar_nav.stopwatch import StopWatch

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Vector3
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, Bool, Int8

TRAFFIC_DETECT_DIST = 20  # [m], 정지선으로부터 몇 m 이내에 들어와야 신호등 인식을 시작할지
TRAFFIC_STOP_DIST = 8    # [m], 정지선으로부터 몇 m 이내에 들어와야 신호등을 통해 정지를 할지
UPDATE_DIST = 3          # [m], 현재 위치와 다음 way의 첫 번째 노드의 거리 => 다음 way로 넘어가는 기준

CHANGE_DIRECTION = ['both', 'left', 'right', 'none']

def euclidean_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        
        # Publisher 설정
        self.path_pub = self.create_publisher(
            Path,
            '/autocar/path',
            10
        )
        
        # Subscriber 설정
        self.goals_sub = self.create_subscription(
            PoseArray,
            '/autocar/goals',
            self.callback_goals,
            10
        )
        
        # 차량 위치 구독
        self.location_sub = self.create_subscription(
            Odometry,
            '/autocar/location',
            self.callback_location,
            10
        )
        
        # 추가 Subscriber
        self.is_avoiding_sub = self.create_subscription(
            Bool,
            '/is_avoiding',
            self.callback_is_avoiding,
            10
        )
        self.traffic_sub = self.create_subscription(
            String,
            '/traffic_sign',
            self.callback_traffic,
            10
        )
        
        # 추가 Publisher
        self.driving_mode_pub = self.create_publisher(
            String,
            '/driving_mode',
            10
        )
        self.traffic_mode_pub = self.create_publisher(
            String,
            '/mode/traffic',
            10
        )
        self.gear_override_pub = self.create_publisher(
            Int8,
            '/gear/override',
            10
        )
        self.speed_maxmin_pub = self.create_publisher(
            Vector3,
            '/speed_maxmin',
            10
        )
        
        # OSM 데이터 초기화
        self.osm_handler = None
        self.ways = {}
        self.way_nodes = {}
        self.ways_info = {}
        self.mission = {}
        self.mission_nodes = {}
        self.mission_types = {}
        self.stopline = {}
        self.stopline_nodes = {}
        self.stopline_way = {}  # 정지선과 way의 매핑
        
        # 전역 경로 저장
        self.global_path = []
        self.current_location = None
        
        # 주행 관련 변수
        self.driving_mode = 'normal_driving'
        self.is_avoiding = False
        self.traffic = {'status': None, 'detected_sign': [], 'target_sign': None}
        self.stopwatch = None
        
        # 지도 로드
        self.load_map()
        
    def load_map(self):
        try:
            # OSM 파일 경로 설정
            osm_file_path = os.path.join(get_package_share_directory('autocar_nav'), 'data')
            osm_files = ["KCITY_MAIN.osm", "KCITY_INTERSECTION_LINK_MAIN.osm"]
            
            # OSM 핸들러 초기화
            self.osm_handler = OSMHandler()
            
            # 각 파일 로드
            for osm_file in osm_files:
                file_path = os.path.join(osm_file_path, osm_file)
                self.get_logger().info(f'파일 로드 중: {osm_file}')
                self.osm_handler.import_file(file_path)
            
            # 데이터 저장
            self.ways = self.osm_handler.ways
            self.way_nodes = self.osm_handler.way_nodes
            self.ways_info = self.osm_handler.ways_info
            self.mission = self.osm_handler.mission_areas
            self.mission_nodes = self.osm_handler.mission_nodes
            self.mission_types = self.osm_handler.mission_types
            self.stopline = self.osm_handler.stopline
            self.stopline_nodes = self.osm_handler.stopline_nodes
            
            # 정지선과 way 매핑 초기화
            self.assign_stopline_way()
            
            self.get_logger().info('모든 지도 파일 로드 완료')
            
        except Exception as e:
            self.get_logger().error(f'지도 로드 실패: {str(e)}')
            
    def assign_stopline_way(self):
        # 선택한 ways에 대한 KDTree (탐색 속도 개선을 위해서)
        selected_ways_kdtree = {}
        self.stopline_way = {}

        for way_id in self.ways:
            waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]]
            selected_ways_kdtree[way_id] = KDTree(waypoints)

        # 각 way에 대해서
        for way_id, tree in selected_ways_kdtree.items():
            closest_stopline = None
            min_distance = np.inf

            # way에서 제일 가까운 stopline 탐색
            for stopline_id, stopline_node in self.stopline.items():
                coordinates = [self.stopline_nodes[v] for v in stopline_node]
                stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*coordinates)]

                distance, _ = tree.query(stp_mid_point)

                if distance > 5:
                    continue
                
                if distance < min_distance:
                    min_distance = distance
                    closest_stopline = stopline_id

            if closest_stopline is not None:
                self.stopline_way[way_id] = closest_stopline

        self.get_logger().info(f'정지선 매핑 완료: {len(self.stopline_way)}개의 매핑')
        
    def callback_location(self, msg):
        self.current_location = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.global_path:
            self.update_local_path()
    
    def callback_goals(self, msg):
        try:
            # 전역 경로 저장
            self.global_path = []
            for pose in msg.poses:
                self.global_path.append((pose.position.x, pose.position.y))
            
            # 로컬 경로 업데이트
            if self.current_location:
                self.update_local_path()
            
        except Exception as e:
            self.get_logger().error(f'경로 변환 실패: {str(e)}')
    
    def callback_is_avoiding(self, msg):
        self.is_avoiding = msg.data
        
    def callback_traffic(self, msg):
        self.traffic['detected_sign'] = [item.strip() for item in msg.data.split(',')]
        
    def update_stopline_dist(self, car_position):
        # 현재 위치에서 가장 가까운 way 찾기
        min_dist = float('inf')
        closest_way = None
        
        for way_id, nodes in self.ways.items():
            for i in range(len(nodes)-1):
                current_pos = self.way_nodes[nodes[i]]
                next_pos = self.way_nodes[nodes[i+1]]
                dist = self.point_to_line_distance(car_position, current_pos, next_pos)
                if dist < min_dist:
                    min_dist = dist
                    closest_way = way_id
        
        if closest_way is None:
            return None, np.inf
            
        # 해당 way의 정지선 찾기
        stopline_id = self.stopline_way.get(closest_way, None)
        if stopline_id is None:
            return None, np.inf
        
        stopline_nodes = self.stopline[stopline_id]
        stp_points = [this.stopline_nodes[v] for v in stopline_nodes]
        stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*stp_points)]
        stp_dist = np.sqrt((stp_mid_point[0] - car_position[0])**2 + (stp_mid_point[1] - car_position[1])**2)
        return stopline_id, stp_dist
        
    def point_to_line_distance(self, point, line_start, line_end):
        # 점과 선분 사이의 최단 거리 계산
        x, y = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 선분의 길이
        line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if line_length == 0:
            return np.sqrt((x - x1)**2 + (y - y1)**2)
        
        # 점에서 선분으로의 투영 비율 계산
        t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / (line_length**2)
        
        if t < 0:
            return np.sqrt((x - x1)**2 + (y - y1)**2)
        elif t > 1:
            return np.sqrt((x - x2)**2 + (y - y2)**2)
        
        # 선분 위의 투영점 계산
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        return np.sqrt((x - proj_x)**2 + (y - proj_y)**2)
        
    def update_driving_mode(self, cur_way):
        driving_mode = 'normal_driving'
        
        if self.is_avoiding:
            driving_mode = 'obstacle_avoiding'
        elif self.ways_info['type'][cur_way] == 1:
            driving_mode = 'intersect'
            
        return driving_mode
        
    def update_traffic(self, distance_stopline):
        gear_override = 0
        
        # Traffic detection 시작 여부 1: 신호 인식 필요 없는 곳은 return
        if self.traffic['target_sign'] == 'no_traffic':
            return 0
        
        if self.traffic['status'] == 'finish':
            return 0
        
        # Traffic detection 시작 여부 2: 정지선으로부터 일정거리 이내 들어오면 인식 시작
        if distance_stopline < TRAFFIC_DETECT_DIST:
            self.get_logger().info('신호등 인식 중')
            self.traffic['status'] = 'detect'
            
        # 정지선으로부터 더 가까워지면 정지 여부 결정
        if distance_stopline < TRAFFIC_STOP_DIST:
            if self.stopwatch is None:
                self.stopwatch = StopWatch()
                self.stopwatch.start()
                
            if self.traffic['target_sign'] == 'left' and ('Left' in self.traffic['detected_sign'] or
                                                       'Straightleft' in self.traffic['detected_sign']):
                gear_override = 0
                self.traffic['status'] = 'finish'
                
            elif self.traffic['target_sign'] == 'right':
                gear_override = 1
                elapsed_time = self.stopwatch.update()
                if elapsed_time > 2.8:
                    gear_override = 0
                    self.traffic['status'] = 'finish'
                    
            elif self.traffic['target_sign'] == 'straight' and ('Green' in self.traffic['detected_sign'] or \
                                                         'Straightleft' in self.traffic['detected_sign'] or \
                                                         'Yellow' in self.traffic['detected_sign']):
                gear_override = 0
                self.traffic['status'] = 'finish'
                
            else:
                gear_override = 1
                
        return gear_override
        
    def update_local_path(self):
        if not self.global_path or not self.current_location:
            return
            
        try:
            # KDTree 생성
            path_kdtree = KDTree(self.global_path)
            
            # 가장 가까운 포인트 찾기
            _, closest_idx = path_kdtree.query(self.current_location)
            
            # 현재 위치에서 가장 가까운 way 찾기
            min_dist = float('inf')
            closest_way = None
            
            for way_id, nodes in self.ways.items():
                for i in range(len(nodes)-1):
                    current_pos = self.way_nodes[nodes[i]]
                    next_pos = self.way_nodes[nodes[i+1]]
                    dist = self.point_to_line_distance(self.current_location, current_pos, next_pos)
                    if dist < min_dist:
                        min_dist = dist
                        closest_way = way_id
            
            # 로컬 경로 포인트 수 설정
            n_back = 2
            n_forward = 15
            
            # 인덱스 범위 계산
            start_idx = max(closest_idx - n_back, 0)
            end_idx = min(closest_idx + n_forward, len(self.global_path))
            
            # 로컬 경로 추출
            local_path = self.global_path[start_idx:end_idx]
            
            # 경로 포인트 보간
            if len(local_path) >= 2:
                dense_path = []
                for i in range(len(local_path)-1):
                    p1 = local_path[i]
                    p2 = local_path[i+1]
                    # 두 점 사이를 5개의 점으로 보간
                    for j in range(5):
                        t = j / 5.0
                        x = p1[0] * (1-t) + p2[0] * t
                        y = p1[1] * (1-t) + p2[1] * t
                        dense_path.append((x, y))
                local_path = dense_path
            
            # Path 메시지 생성
            path_msg = Path()
            path_msg.header.frame_id = "world"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # PoseStamped 메시지 생성
            for i in range(len(local_path)-1):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "world"
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                
                # 위치 설정
                pose_stamped.pose.position.x = local_path[i][0]
                pose_stamped.pose.position.y = local_path[i][1]
                
                # 방향 계산 (현재 점에서 다음 점을 향하도록)
                dx = local_path[i+1][0] - local_path[i][0]
                dy = local_path[i+1][1] - local_path[i][1]
                yaw = np.arctan2(dy, dx)
                
                quaternion = yaw_to_quaternion(yaw)
                pose_stamped.pose.orientation.x = quaternion.x
                pose_stamped.pose.orientation.y = quaternion.y
                pose_stamped.pose.orientation.z = quaternion.z
                pose_stamped.pose.orientation.w = quaternion.w
                
                path_msg.poses.append(pose_stamped)
            
            # 마지막 포인트 추가
            if local_path:
                last_pose = PoseStamped()
                last_pose.header.frame_id = "world"
                last_pose.header.stamp = self.get_clock().now().to_msg()
                last_pose.pose.position.x = local_path[-1][0]
                last_pose.pose.position.y = local_path[-1][1]
                
                if len(local_path) > 1:
                    dx = local_path[-1][0] - local_path[-2][0]
                    dy = local_path[-1][1] - local_path[-2][1]
                    yaw = np.arctan2(dy, dx)
                    quaternion = yaw_to_quaternion(yaw)
                    last_pose.pose.orientation = pose_stamped.pose.orientation
                
                path_msg.poses.append(last_pose)
            
            # 주행 모드 업데이트
            if closest_way is not None:
                driving_mode = self.update_driving_mode(closest_way)
                self.driving_mode_pub.publish(String(data=str(driving_mode)))
            
            # 정지선과의 최소 거리 업데이트
            _, min_dist_stopline = self.update_stopline_dist(self.current_location)
            
            # 신호등 상태 확인 및 기어 오버라이드 값 설정
            gear_override = self.update_traffic(min_dist_stopline)
            
            # 기어 오버라이드와 신호 상태 publish
            self.gear_override_pub.publish(Int8(data=gear_override))
            self.traffic_mode_pub.publish(String(data=str(self.traffic['status'])))
            
            # 경로 발행
            self.path_pub.publish(path_msg)
            self.get_logger().info(f'로컬 경로 발행 완료: {len(path_msg.poses)}개의 포인트')
            
        except Exception as e:
            self.get_logger().error(f'로컬 경로 생성 실패: {str(e)}')

def main():
    rclpy.init()
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()