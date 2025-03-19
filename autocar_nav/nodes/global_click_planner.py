#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree

from autocar_nav.stopwatch import StopWatch
from autocar_nav.osm_handler import OSMHandler
from autocar_nav.keyboard_input import KeyboardInput
from autocar_nav.way_selector import WaySelector
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion

from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool, Int8

from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

TRAFFIC_DETECT_DIST = 20 # [m], 정지선으로부터 몇 m 이내에 들어와야 신호등 인식을 시작할지
TRAFFIC_STOP_DIST = 8 # [m], 정지선으로부터 몇 m 이내에 들어와야 신호등을 통해 정지를 할지

NEXT_WAY_DIST = 3 # [m], 현재 위치와 다음 way의 첫 번째 노드의 거리 => 다음 way로 넘어가는 기준

CHANGE_DIRECTION = ['both', 'left', 'right', 'none']

# ==================================================================================
import pyproj
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def euclidean_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
# ==================================================================================

msg = """
======= Global path planning =========

Left mouse click: 경로 선택

q: 최근에 선택한 경로 취소

w: 선택했던 경로 다시 Publish

Backspace: 모든 선택한 경로 취소

======================================
"""
osmhandler = OSMHandler()
OSM_FILE_PATH = os.path.join(get_package_share_directory('autocar_nav'), 'data')
OSM_FILE_LIST = ["hitech2_LINK.osm", "hitech2_INTERSECTION_LINK.osm", "hitech2_STOPLINE.osm", "boong_LINK.osm"]
#OSM_FILE_LIST = ["KCITY_MAIN.osm","KCITY_INTERSECTION_LINK_MAIN.osm"]
# import the osm files
for osm_file in OSM_FILE_LIST:
    osmhandler.import_file(OSM_FILE_PATH + '/' + osm_file)
# Attributes for ways and nodes
# self.ways = osmhandler.ways # {way1: [node1, node2, ...], way2:[node11,node12,...]}
# self.way_nodes = osmhandler.way_nodes # {node1:[x1,y1], node2:[x2,y2], ... }

class GlobalPathPlanning(Node):
    def __init__(self):
        super().__init__('global_path_planning')

        # =================== Way selector =======================
        # Subscribers
        self.clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point',  self.callback_selecting_ways_by_clicking, 10)

        # Publishers
        self.way_pub = self.create_publisher(PoseArray, '/ways',  10) # 모든 ways
        self.mission_areas_pub = self.create_publisher(MarkerArray, '/mission_areas',  10) # 미션구역
        self.stoplines_pub = self.create_publisher(MarkerArray, '/stoplines',  10) # 미션구역
        self.clicked_way_pub = self.create_publisher(PoseArray, '/clicked_closest_way',  10) # 해당 지점에서 제일 가까운 way
        self.candidate_ways_pub = self.create_publisher(PoseArray, '/candidates_ways', 10) # 제일 가까운 way 다음의 후보 ways
        self.selected_ways_pub = self.create_publisher(PoseArray,'/autocar/goals',  10) # target ways /selected_ways
        
        # =================== 주행 중 =======================
        # Subscribers
        #self.location_sub = self.create_subscription(State2D, '/autocar/state2D', self.callback_location, 10) # 종방향 에러 계산할 때
        self.location_sub = self.create_subscription(Odometry, '/autocar/location', self.callback_location, 10) # 종방향 에러 계산할 때

        # self.is_avoiding_sub = self.create_subscription(Bool,'/is_avoiding', self.callback_is_avoiding, 10)
        # self.traffic_sub = self.create_subscription(String, '/traffic_sign',  self.callback_traffic, 10)

        # Publishers
        self.closest_waypoints_pub = self.create_publisher(PoseArray,'/global_closest_waypoints',  10)
        self.waypoints_pub = self.create_publisher(PoseArray, '/global_waypoints', 10)
        # self.driving_mode_pub = self.create_publisher(String, '/driving_mode', queue_size=10)
        # self.traffic_mode_pub = self.create_publisher(String, '/mode/traffic',  queue_size=10)
        # self.gear_override_pub = self.create_publisher(Int8, '/gear/override',  queue_size=10)

        # self.cluster_ROI_pub = self.create_publisher(String, '/cluster_ROI',  queue_size=10)
        # self.speed_maxmin_pub = self.create_publisher(Vector3, '/speed_maxmin',  queue_size=10)
        # self.way_change_pub = self.create_publisher(Bool, '/way_change_signal',  queue_size=10)
        # Timers
        self.timer_driving = self.create_timer(0.1, self.callback_timer_driving)
        self.timer_selecting = self.create_timer(0.1, self.callback_timer_selecting_ways_by_key_input)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # =================== set attributes =======================
        self.is_published_once = False
        self.selected_ways = None
        self.location = None
        self.stopwatch = None

        # Driving에서 쓰이는 변수들
        self.prev_way = None
        self.possible_change_direction = 'both'
        self.cur_way = {'id':None,
                        'change_direction':None,
                        'idx':0,
                        'stopline_id':None,
                        'waypoints':None,
                        'path':None,
                        'cluster_ROI':None,
                        'speed_max':2.5,
                        'speed_min':2.0}

        # traffic light
        self.traffic = {'status':None, 'detected_sign':[], 'target_sign':None}

        self.driving_mode = 'normal_driving' # ['normal_driving', 'obstacle_avoiding', 'intersect', 'parking', 'delivery']
        self.is_avoiding = False

        # OSM Handler
        osmhandler = OSMHandler()

        # import the osm files
        for osm_file in OSM_FILE_LIST:
            osmhandler.import_file(OSM_FILE_PATH + '/' + osm_file)

        # Attributes for ways and nodes
        self.ways = osmhandler.ways # {way1: [node1, node2, ...], way2:[node11,node12,...]}
        self.way_nodes = osmhandler.way_nodes # {node1:[x1,y1], node2:[x2,y2], ... }
        self.ways_info = osmhandler.ways_info
        # print(self.ways_info)
        self.mission_way = {}

        self.mission = osmhandler.mission_areas
        self.mission_nodes = osmhandler.mission_nodes
        self.mission_types = osmhandler.mission_types

        self.stopline = osmhandler.stopline
        self.stopline_nodes = osmhandler.stopline_nodes

        # WaySelector
        self.way_selector = WaySelector(self.ways, self.way_nodes)

        # Key Input
        self.keyboard_input = KeyboardInput(rate=0.1)

        print(msg)
        print("======== 불러온 목록 ==========")
        print(f"주행유도선 노드: {len(self.way_nodes)}개")
        print(f"미션 구역: {len(self.mission)}개")
        print(f"정지선: {len(self.stopline)}개")
        print("============================")

    def callback_timer_driving(self):
    # 최초 한 번만 모든 객체를 publish하도록 설정
        if self.is_published_once == False:
                if self.way_pub.get_subscription_count() == 1:  # 구독자가 1명인지 확인
                    self.publish_all_objects()  # 모든 객체를 publish
                    self.is_published_once = True  # 이후 다시 publish하지 않도록 플래그 설정
                return
        
        # 아직 주행 준비가 되지 않았으면 함수 종료
        if self.location is None or len(self.way_selector.selected_ways) == 0:
            return
        
        # 현재 위치를 기준으로 현재 진행 중인 경로(cur_way) 업데이트
        self.update_current_way(self.location)

        # 만약 현재 진행 중인 경로가 이전 경로와 다르다면, 정보 업데이트 및 publish 수행
        if self.cur_way['id'] != self.prev_way:
            self.prev_way = self.cur_way['id']  # 이전 경로 갱신
            
            # 현재 경로의 차선 변경 방향, 경로 정보, ROI 영역, 최대/최소 속도 등 업데이트
            self.cur_way['change_direction'] = CHANGE_DIRECTION[self.ways_info['change_direction'][self.cur_way['id']]]
            self.cur_way['path'] = self.ways_info['path'][self.cur_way['id']]
            self.cur_way['cluster_ROI'] = self.ways_info['cluster_ROI'][self.cur_way['id']]
            self.cur_way['speed_max'] = self.ways_info['speed_max'][self.cur_way['id']]
            self.cur_way['speed_min'] = self.ways_info['speed_min'][self.cur_way['id']]

            # 신호등 정보 초기화 및 목표 신호등 정보 설정
            self.traffic['status'] = None
            self.traffic['target_sign'] = self.ways_info['traffic'][self.cur_way['id']]
            self.stopwatch = None  # 신호등 관련 타이머 초기화
            
            # 현재 경로의 웨이포인트 생성
            self.cur_way['waypoints'] = self.get_waypoints()

            # 생성한 웨이포인트 및 관련 정보를 메시지로 변환하여 publish
            global_waypoints_msg = self.make_waypoints_msg(
                self.cur_way['waypoints'], 
                self.cur_way['change_direction'],
                self.cur_way['path']
            )
            self.waypoints_pub.publish(global_waypoints_msg)  # 전체 웨이포인트 publish
            
            #self.cluster_ROI_pub.publish(self.cur_way['cluster_ROI'])  # 클러스터 ROI 정보 publish

            #self.way_change_pub.publish(True)  # 경로 변경 이벤트 publish

            # 최대/최소 속도 정보를 메시지로 변환하여 publish
            speed_maxmin_msg = Vector3(x=self.cur_way['speed_max'], y=self.cur_way['speed_min'])
            #self.speed_maxmin_pub.publish(speed_maxmin_msg)

        # 현재 주행 모드 업데이트
        driving_mode = self.update_driving_mode(self.cur_way['id'])

        # 정지선과의 최소 거리 업데이트
        _, min_dist_stopline = self.update_stopline_dist(self.location)
        
        # 신호등 상태 확인 및 기어 오버라이드 값 설정
        #gear_override = self.update_traffic(min_dist_stopline) 

        # 주변 웨이포인트 추출
        near_waypoints, _ = self.get_near_waypoints()
        
        # ROS 토픽으로 주행 모드, 기어 오버라이드, 신호 상태 publish
        # self.driving_mode_pub.publish(driving_mode)
        # self.gear_override_pub.publish(gear_override)
        # self.traffic_mode_pub.publish(self.traffic['status'])

        # 주변 웨이포인트를 메시지로 변환하여 publish (시각화를 위한 용도)
        near_waypoints_msg = self.make_waypoints_msg(near_waypoints, 'utm', 'none')
        self.closest_waypoints_pub.publish(near_waypoints_msg)
        return
    
    def callback_location(self, location_msg):
        self.location = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.x)
        #self.location = (location_msg.pose.x, location_msg.pose.y)
        return
    
    # def callback_is_avoiding(self, is_avoiding_msg):
    #     self.is_avoiding = is_avoiding_msg.data
    #     return
    
    # def callback_traffic(self, traffic_msg):
    #     self.traffic['detected_sign'] = [item.strip() for item in traffic_msg.data.split(',')]  
    #     # self.traffic_detected = [item.strip() for item in traffic_msg.data.split(',')]  
    #     return
    
    def update_stopline_dist(self, car_position):
        stopline_id = self.stopline_way.get(self.cur_way['id'], None)
        # print("정지선:", stopline_id)
        if stopline_id is None:
            return None, np.inf
        
        stopline_nodes = self.stopline[stopline_id]
        stp_points = [self.stopline_nodes[v] for v in stopline_nodes]
        stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*stp_points)]
        stp_dist = np.sqrt((stp_mid_point[0] - car_position[0])**2 + (stp_mid_point[1] - car_position[1])**2)
        return stopline_id, stp_dist
    
    # def update_mission_status(self, cur_way, car_pos):
    #     cur_mission_id = self.mission_way.get(cur_way, None)

    #     if cur_mission_id is None:
    #         return None, False
        
    #     mission_type = self.mission_types[cur_mission_id]
    #     mission_node_ids = self.mission[cur_mission_id]
        
    #     mission_nodes = [self.mission_nodes[node_id] for node_id in mission_node_ids[:4]]

    #     is_inside = self.is_point_in_rectangle(mission_nodes, car_pos)

    #     return mission_type, is_inside
    
    # def is_point_in_rectangle(self, rect_points, point):
    #     def cross_product(o, a, b):
    #         return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        
    #     A, B, C, D = rect_points

    #     cp1 = cross_product(A, B, point)
    #     cp2 = cross_product(B, C, point)
    #     cp3 = cross_product(C, D, point)
    #     cp4 = cross_product(D, A, point)
        
    #     return (cp1 >= 0 and cp2 >= 0 and cp3 >= 0 and cp4 >= 0) or (cp1 <= 0 and cp2 <= 0 and cp3 <= 0 and cp4 <= 0)
    
    def update_driving_mode(self, cur_way):
        driving_mode = 'normal_driving'
        # driving_mode = None

        # # cur way에 대해서 미션 구역인지 판단 & 미션 구역 내에 들어왔는지 판단
        # mission_type, is_inside_mission = self.update_mission_status(cur_way, self.location)
        # # print('current mission type:', mission_type, '\n')
        # if is_inside_mission == True:
        #     driving_mode = mission_type

        # elif self.is_avoiding == True:
        #     driving_mode = 'obstacle_avoiding'
        
        # elif self.ways_info['type'][cur_way] == 1:
        #     driving_mode = 'intersect'

        # else:
        #     driving_mode = 'normal_driving'

        return driving_mode
        
    def update_current_way(self, position):
        self.cur_way['id'] = self.way_selector.selected_ways[self.cur_way['idx']]
        
        # 마지막 노드일 때
        if self.cur_way['id'] == self.way_selector.selected_ways[-1]:
            return self.cur_way['id']
        
        next_way = self.ways[self.way_selector.selected_ways[self.cur_way['idx']+1]]
        next_way_start_node = self.way_nodes[next_way[0]]

        next_dist = euclidean_distance(position, next_way_start_node)

        if next_dist < NEXT_WAY_DIST: # 다음 way 첫 노드로부터 3m 이내에 들어오면 다음 way로 넘어감
            self.cur_way['idx'] += 1
            self.prev_way = self.cur_way['id']

        return self.cur_way['id']
    
    # def update_traffic(self, distance_stopline):
    #     gear_override = 0

    #     # Traffic detection 시작 여부 1: 신호 인식 필요 없는 곳은 return
    #     if self.traffic['target_sign'] == 'no_traffic':
    #         return 0
        
    #     if self.traffic['status'] == 'finish':
    #         return 0
        
    #     # Traffic detection 시작 여부 2: 정지선으로부터 일정거리 이내 들어오면 인식 시작
    #     if distance_stopline < TRAFFIC_DETECT_DIST:
    #         print('신호등 인식 중')
    #         self.traffic['status'] = 'detect'

    #     # 정지선으로부터 더 가까워지면 정지 여부 결정
    #     if distance_stopline < TRAFFIC_STOP_DIST:
    #         if self.stopwatch is None:
    #             self.stopwatch = StopWatch()
    #             self.stopwatch.start()

    #         print('traffic_detected:', self.traffic['detected_sign'])
    #         print('target sign: ',self.traffic['target_sign'])
    #         if self.traffic['target_sign'] == 'left' and ('Left' in self.traffic['detected_sign'] or
    #                                                        'Straightleft' in self.traffic['detected_sign']): #'Straightleft'
    #             gear_override = 0
    #             self.traffic['status'] = 'finish'

    #         elif self.traffic['target_sign'] == 'right':
    #             gear_override = 1

    #             elapsed_time = self.stopwatch.update()
    #             if elapsed_time > 2.8:
    #                 gear_override = 0
    #                 self.traffic['status'] = 'finish'

    #         elif self.traffic['target_sign'] == 'straight' and ('Green' in self.traffic['detected_sign'] or \
    #                                                     'Straightleft' in self.traffic['detected_sign'] or \
    #                                                     'Yellow' in self.traffic['detected_sign']):
    #             gear_override = 0
    #             self.traffic['status'] = 'finish'

    #         else:
    #             gear_override = 1
    #     return gear_override
    
    def get_waypoints(self):
        # waypoints
        waypoints = []
    
        # # 1. near_way 찾기
        # start_index = max(self.cur_way['idx'] - 1, 0) # 시작 인덱스와 끝 인덱스 계산
        # end_index = min(self.cur_way['idx'] + 1, len(self.way_selector.selected_ways))
        # near_ways = self.way_selector.selected_ways[start_index:end_index + 1] # 인근한 3개의 way 추출
        
        cur_waypoints = [self.way_nodes[node_id] for node_id in self.ways[self.cur_way['id']]]
        if self.cur_way['idx'] == 0:
            print("1")
            if len(self.way_selector.selected_ways) == 1:
                waypoints = cur_waypoints
            
            else:
                next_way_id = self.way_selector.selected_ways[self.cur_way['idx']+1]
                next_waypoints = [self.way_nodes[node_id] for node_id in self.ways[next_way_id]]
                end_len = min(len(next_waypoints), 7)
                waypoints = cur_waypoints +\
                            next_waypoints[:end_len-1]
        
        elif self.cur_way['idx'] == len(self.way_selector.selected_ways)-1:
            print("2")
            prev_way_id = self.way_selector.selected_ways[self.cur_way['idx']-1]
            prev_waypoints = [self.way_nodes[node_id] for node_id in self.ways[prev_way_id]]
            start_len = min(len(prev_waypoints), 7)
            waypoints = prev_waypoints[len(prev_waypoints)-start_len:] +\
                        cur_waypoints 
        else:
            print("3")
            prev_way_id = self.way_selector.selected_ways[self.cur_way['idx']-1]
            prev_waypoints = [self.way_nodes[node_id] for node_id in self.ways[prev_way_id]]
            start_len = min(len(prev_waypoints), 7)

            next_way_id = self.way_selector.selected_ways[self.cur_way['idx']+1]
            next_waypoints = [self.way_nodes[node_id] for node_id in self.ways[next_way_id]]
            end_len = min(len(next_waypoints), 7)
            
            waypoints = prev_waypoints[len(prev_waypoints)-start_len:] +\
                        cur_waypoints +\
                        next_waypoints[:end_len-1]
    
        # 2. 인근 ways로부터 waypoints 추출
        # for way_id in near_ways:
        #     waypoints += [self.way_nodes[node_id] for node_id in self.ways[way_id]]
        return waypoints
    
    def get_near_waypoints(self):
        waypoints = self.cur_way['waypoints']
        
        waypoints_kdtree = KDTree(waypoints)
        _, closest_node_idx = waypoints_kdtree.query(self.location)

        # 가장 가까운 노드로부터 앞으로 15개, 뒤로 4개 찾기
        n_back = 4
        n_forward = 10
        start_index = max(closest_node_idx - n_back, 0)
        end_index = min(closest_node_idx + n_forward, len(waypoints))
        
        # 인근 waypoints 추출
        near_waypoints = waypoints[start_index:end_index + 1]

        return near_waypoints, closest_node_idx

    # way에 미션 구역 할당하기
    # def assign_mission_way(self, selected_ways):
    #     # 선택한 ways에 대한 KDTree (탐색 속도 개선을 위해서)
    #     selected_ways_kdtree = {}
    #     self.mission_way = {}

    #     for way_id in selected_ways:
    #         waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]] # [(x1, y1), (x2, y2), ...]
    #         selected_ways_kdtree[way_id] = KDTree(waypoints)

    #     # 각 미션 구역에 대
    #     for mission_id, misson_node_ids in self.mission.items():
    #         # 미션 구역 중심 찾기
    #         mission_cp = np.sum(np.array([self.mission_nodes[mn_id] for mn_id in misson_node_ids[:4]]), axis=0) / 4

    #         # 미션 구역과 제일 가까운 way 탐색
    #         closest_way = None
    #         min_distance = np.inf
    #         for way_id, tree in selected_ways_kdtree.items():
    #             distance, _ = tree.query(mission_cp)
                
    #             if distance > 5:
    #                 continue
                
    #             if distance < min_distance:
    #                 min_distance = distance
    #                 closest_way = way_id

    #         if closest_way is not None:
    #             self.mission_way[closest_way] = mission_id

    #     print('mission_way: ',self.mission_way)
    #     return
    
    # way에 stopline 할당하기
    def assign_stopline_way(self, selected_ways):
        # 선택한 ways에 대한 KDTree (탐색 속도 개선을 위해서)
        selected_ways_kdtree = {}
        self.stopline_way = {}

        for way_id in selected_ways:
            # if self.ways_info['type'][way_id] != 0 :
            #     continue
            waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]] # [(x1, y1), (x2, y2), ...]
            selected_ways_kdtree[way_id] = KDTree(waypoints)

        # 각 미션 구역에 대
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

        print('stopline_way: ',self.stopline_way)
        return
    
    # ==== Way Selecting =======================================================
    def callback_selecting_ways_by_clicking(self, clicked_point_msg):
        # Rviz 상에서 마우스 좌클릭한 위치 반환
        #clicked_point = latlon_to_utm(clicked_point_msg.point.y, clicked_point_msg.point.x)

        # base_link → utm 변환 가져오기
        transform = self.tf_buffer.lookup_transform('world', clicked_point_msg.header.frame_id, rclpy.time.Time())
        # 좌표 변환
        clicked_point = tf2_geometry_msgs.do_transform_point(clicked_point_msg, transform)
        self.get_logger().info(f"Clicked point: {clicked_point.point.x}, {clicked_point.point.y}")

        # way_selectotor - clicked_point_list 업데이트
        clicked_way, candidate_ways, recently_selected_ways =\
                            self.way_selector.update_click_input(clicked_point)
        
        print("현재 선택된 way: ", self.way_selector.selected_ways)

        # 미션 구역 중에서 제일 가까운 way 찾기
        # self.assign_mission_way(self.way_selector.selected_ways)
        self.assign_stopline_way(self.way_selector.selected_ways)
        
        # ROS msg 생성 및 Publish
        clicked_way_msg = self.make_way_msg([clicked_way])
        candidate_ways_msg = self.make_way_msg(candidate_ways)
        selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)

        self.clicked_way_pub.publish(clicked_way_msg)
        self.candidate_ways_pub.publish(candidate_ways_msg)
        self.selected_ways_pub.publish(selected_ways_msg)

        self.prev_way = None
        return

    def callback_timer_selecting_ways_by_key_input(self):
        key_input = self.keyboard_input.update()
        key_input_list = ['\x7f', 'q', 'w', 'a', '\x03']

        if key_input not in key_input_list:
            return 
        
        if key_input == '\x7f': # Backspace
            self.way_selector.reset_selected_ways()
            selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)
            self.selected_ways_pub.publish(selected_ways_msg)

            clicked_way_msg = self.make_way_msg([])
            self.clicked_way_pub.publish(clicked_way_msg)

            candidate_ways_msg = self.make_way_msg([])
            self.candidate_ways_pub.publish(candidate_ways_msg)

        elif key_input == 'q':
            self.way_selector.remove_target_ways()
            selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)
            self.selected_ways_pub.publish(selected_ways_msg)

            clicked_way_msg = self.make_way_msg([])
            self.clicked_way_pub.publish(clicked_way_msg)

            candidate_ways_msg = self.make_way_msg([])
            self.candidate_ways_pub.publish(candidate_ways_msg)

        elif key_input == 'w':
            selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)
            self.selected_ways_pub.publish(selected_ways_msg)
        
        elif key_input == 'a':
            self.publish_all_objects()

        elif key_input == '\x03': # Ctrl+C
            self.get_logger().info("Ctrl+C has been pressed. Shutting down...")
            rclpy.shutdown()
        
        print("현재 선택된 way: ", self.way_selector.selected_ways)
        return

    def publish_all_objects(self):
        print("All objects are published.")
        print("")
        all_ways = self.make_way_msg(self.ways)
        self.way_pub.publish(all_ways)

        all_mission_areas = self.make_mission_msg(self.mission)
        self.mission_areas_pub.publish(all_mission_areas)
        
        all_stoplines = self.make_stopline_msg(self.stopline)
        self.stoplines_pub.publish(all_stoplines)
        return 
    
    # ROS
    def make_waypoints_msg(self, waypoints, possible_change_direction, path):
        waypoints_msg = PoseArray()
        waypoints_msg.header.frame_id = possible_change_direction

        for i in range(len(waypoints)-1):
            pose = Pose()
            pose.position.x = waypoints[i][0]
            pose.position.y = waypoints[i][1]

            # 방향 계산
            yaw = np.arctan2(waypoints[i+1][1] - waypoints[i][1], waypoints[i+1][0] - waypoints[i][0])
            quaternion = yaw_to_quaternion(yaw)
            pose.orientation.x = quaternion.x
            pose.orientation.y = quaternion.y
            pose.orientation.z = quaternion.z
            pose.orientation.w = quaternion.w

            if path == 'normal':
                pose.position.z = 0.0
            elif path == 'frenet':
                pose.position.z = 1.0
            else:
                pose.position.z = 0.0
        
            waypoints_msg.poses.append(pose)
        return waypoints_msg
    
    # ROS
    def make_way_msg(self, ways_for_visualize):
        ways = PoseArray()
        ways.header.frame_id = "world"
        ways.header.stamp = self.get_clock().now().to_msg()

        for way_id in ways_for_visualize:
            position_prev = None
            for node_id in self.ways[way_id]:
                position = self.way_nodes[node_id]

                # 첫 번째 점이면 continue
                if position_prev is None:
                    position_prev = position
                    continue
                
                # 방향 계산
                yaw = np.arctan2(position[1] - position_prev[1], position[0] - position_prev[0])
                quaternion = yaw_to_quaternion(yaw)

                # Pose 생성
                pose = Pose()
                pose.position.x, pose.position.y = position
                pose.orientation.x = quaternion.x
                pose.orientation.y = quaternion.y
                pose.orientation.z = quaternion.z
                pose.orientation.w = quaternion.w
                ways.poses.append(pose)
                position_prev = position # 현재 위치 정보 업데이트
        
        return ways
    
    # ROS
    def make_mission_msg(self, areas_for_visualize):
        mission = MarkerArray()

        for area_id in areas_for_visualize:
            polygon = Marker()
            polygon.header.frame_id = 'world'
            polygon.type = Marker.LINE_STRIP
            polygon.action = Marker.ADD
            polygon.id = area_id

            # Marker scale
            polygon.scale.x = 1.0
            polygon.scale.y = 1.0

            # Marker color
            polygon.color.a = 1.0
            polygon.color.r = 0.0
            polygon.color.g = 1.0
            polygon.color.b = 1.0

            x_centers, y_centers = [], []
            for node_id in self.mission[area_id]:
                position = self.mission_nodes[node_id]
                p = Point()
                p.x = position[0]
                p.y = position[1]
                polygon.points.append(p)

                x_centers.append(p.x)
                y_centers.append(p.y)

            mission.markers.append(polygon)

            x_center = np.array(x_centers[:-1]).sum() / (len(self.mission[area_id])-1)
            y_center = np.array(y_centers[:-1]).sum() / (len(self.mission[area_id])-1)
            
            text_marker = Marker()
            text_marker.header.frame_id = 'world'
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.id = area_id + 10000

            text_marker.pose.position.x = x_center  # x 위치
            text_marker.pose.position.y = y_center  # y 위치
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3 # text scale, mapviz에서 적용이 안됨

            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0

            text_marker.text = self.mission_types[area_id]
            mission.markers.append(text_marker)

        return mission
    
    # ROS
    def make_stopline_msg(self, stopline_for_visualize):
        stoplines = MarkerArray()
        
        for stopline_id, stopline_node in stopline_for_visualize.items():
            
            positions = [self.stopline_nodes[v] for v in stopline_node]
            
            stopline = Marker()
            stopline.header.frame_id = "world"
            stopline.type = Marker.LINE_STRIP
            stopline.action = Marker.ADD
            stopline.id = stopline_id
            
            stopline.scale.x = 0.1
            
            stopline.color.a = 1.0
            stopline.color.r = 1.0
            stopline.color.g = 0.0
            stopline.color.b = 0.0
            
            for position in [positions[0], positions[-1]]:
                p = Point()
                p.x = position[0]
                p.y = position[1]
                stopline.points.append(p)
                
            stoplines.markers.append(stopline) 
 
        return stoplines
    
def main(args=None):
    # Initialise the node
    rclpy.init(args=args)
    
    try:
        # Initialise the class
        global_path_planning = GlobalPathPlanning()

        # Stop the node from exiting
        rclpy.spin(global_path_planning)

    finally:
        global_path_planning.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()

