#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from planning_msgs.msg import ModeState
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import numpy as np
from visualization_msgs.msg import Marker
import csv
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import math
from geometry_msgs.msg import Quaternion

# Frenet optimal trajectory imports
from .frenet_planner import frenet_optimal_trajectory as fot
from .frenet_planner import cubic_spline_planner
# Hybrid A* imports
from .hybrid_astar.hybrid_a_star import hybrid_a_star_planning


class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        # Subscriber (/mode_state)
        self.create_subscription(ModeState, '/mode_state', self.mode_state_callback, 10)
        self.create_subscription(PointCloud2, '/obstacle_detector', self.obstacle_callback, 10)
        self.create_subscription(Odometry, '/localization', self.localization_callback, 10)
        self.create_subscription(Path, '/global_waypoints', self.global_waypoints_callback, 10)
        self.create_subscription(Point, '/stopline_detector', self.stopline_callback, 10)
        self.create_subscription(String, '/traffic_detector', self.traffic_callback, 10)
        self.create_subscription(PoseStamped, '/hybrid_astar_goal', self.goal_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_zone', self.obstacle_zone_callback, 10)
        self.create_subscription(PoseArray, '/delivery_zone', self.delivery_zone_callback, 10)
        self.create_subscription(PoseArray, '/unload_zone', self.unload_zone_callback, 10)
        self.create_subscription(PoseArray, '/parking_zone', self.parking_zone_callback, 10)
        # --- 미션 리셋 토픽 구독 ---
        self.create_subscription(String, '/reset_missions', self.reset_missions_callback, 10)

        # Publisher (/local_path)
        self.path_publisher = self.create_publisher(PoseArray, '/local_path', 10)
        self.stopline_marker_pub = self.create_publisher(Marker, '/stopline_marker', 10)

        # 상태 변수 초기화
        self.current_mode = ModeState.DRIVE
        self.obstacle_data = None
        self.current_pose = None
        self.global_waypoints = None
        self.stopline_position = None
        self.goal_pose = None

        # 상태 변수 추가
        self.current_traffic_light = "RED"

        # Marker Publisher 추가
        self.global_marker_publisher = self.create_publisher(Marker, '/global_path_marker', 10)
        self.marker_publisher = self.create_publisher(Marker, '/local_path_marker', 10)

        # 초기 모드
        self.current_mode = ModeState.DRIVE

        # 타이머 설정 (1초 주기)
        self.timer = self.create_timer(1.0, self.publish_local_path)

        # Frenet 초기화 (예제 값)
        self.init_frenet_planner()
        self.get_logger().info('Local Planner Node Initialized with Perception & Localization')

        # --- 배달 미션 상태머신/플래그 ---
        self.delivery_state = 'NONE'  # NONE, ENTRY, WAIT, EXIT, DONE
        self.delivery_wait_timer = 0.0
        self.delivery_zone_done = False
        # 배달구역 좌표(더미, 실제는 외부에서 받아야 함)
        self.delivery_entry = (0, 95)
        self.delivery_center = (10, 105)
        self.delivery_exit = (10, 125)

        # --- 주차 미션 상태머신/플래그 ---
        self.parking_state = 'NONE'  # NONE, ENTRY, WAIT, EXIT, DONE
        self.parking_wait_timer = 0.0
        self.parking_zone_done = False
        # 주차구역 좌표(더미, 실제는 외부에서 받아야 함)
        self.parking_entry = (200, 225)
        self.parking_center = (200, 275)
        self.parking_exit = (200, 325)

        # --- 장애물 회피 미션 상태머신/플래그 ---
        self.obstacle_state = 'NONE'  # NONE, AVOID, RECOVER, DONE
        self.obstacle_zone_done = False
        # 장애물 회피 시작점/복귀점(더미)
        self.obstacle_avoid_entry = (50, 345)
        self.obstacle_avoid_recover = (70, 355)

        # --- 정지선 미션 상태머신/플래그 ---
        self.stopline_state = 'NONE'  # NONE, WAIT, DONE
        self.stopline_wait_timer = 0.0
        self.stopline_zone_done = False
        # 정지선 중심 좌표(더미, 실제는 외부에서 받아야 함)
        self.stopline_centers = []
        self.stopline_flags = {}  # {(x, y): bool}
        self.current_stopline_id = None

         # --- 미션 상태 퍼블리셔 ---
        self.mission_state_pub = self.create_publisher(String, '/mission_state', 10)

        # --- 정지선/장애물 플래그를 좌표 기반 ID(튜플)로 딕셔너리 관리 ---
        self.obstacle_flags = {}  # {(x, y): bool}
        self.current_obstacle_id = None

        # --- 하차 미션 상태머신 추가 ---
        self.unload_state = 'NONE'  # NONE, ENTRY, WAIT, EXIT, DONE
        self.unload_wait_timer = 0.0
        self.unload_zone_done = False
        # 하차구역 좌표(더미, 실제는 외부에서 받아야 함)
        self.unload_entry = (10, 255)
        self.unload_center = (10, 275)
        self.unload_exit = (10, 295)

        # --- 장애물 영역(PoseArray) 구독 및 저장 ---
        self.obstacle_zones = []  # (type, params) 튜플 리스트: ('circle', (x, y, r)), ('polygon', [(x1,y1),...])
        # --- 구역 Polygon 저장용 ---
        self.delivery_zone_polygon = []
        self.unload_zone_polygon = []
        self.closed_parking_zones = []  # 여러 Polygon
        self.open_parking_segment = []  # 선분

        # 추가된 변수
        self.last_generated_state = None
        self.last_valid_path = None

    def get_yaw_from_quaternion(self, q: Quaternion):
        """
        쿼터니언을 Yaw(heading) 값으로 변환합니다.
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # CALLBACK
    # 선택된 모드 callback
    def mode_state_callback(self, msg):
        self.current_mode = msg.current_mode
        self.get_logger().info(f'Received Mode: {msg.description}')
    # 장애물 인식기기 데이터 callback
    def obstacle_callback(self, msg: PointCloud2):
        # 장애물 좌표를 ID로 사용
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            id = (round(x, 2), round(y, 2))
            if id not in self.obstacle_flags:
                self.obstacle_flags[id] = False
        self.obstacle_data = msg
        self.get_logger().info('Obstacle data received')
    # Localization 데이터 callback
    def localization_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.get_logger().info('Localization data updated')
    # 정지선 callback
    def stopline_callback(self, msg: Point):
        self.stopline_position = msg
        self.get_logger().info(f'Stopline position updated: {msg.x}, {msg.y}')
    # 신호등 callback
    def traffic_callback(self, msg: String):
        self.current_traffic_light = msg.data
        self.get_logger().info(f'Traffic light updated: {self.current_traffic_light}')
    # Global waypoints callback 글로벌 경로 수신
    def global_waypoints_callback(self, msg: Path):
        self.global_waypoints = msg
        self.get_logger().info('Global waypoints received')
    # 장애물 콜백
    def obstacle_callback(self, msg: PointCloud2):
        self.get_logger().info('Obstacle callback triggered')
        self.obstacle_data = msg
    # 목표 콜백
    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg.pose
        self.get_logger().info(
            f"[HybridA*] Received goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
    # 기능 함수
    # 장애물 위치 파악
    def get_obstacle_positions(self, cloud_msg: PointCloud2):
        if cloud_msg is None:
            return [(30.0, 5.0), (35.0, 6.0)]  # 테스트용 더미
        obstacle_positions = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            obstacle_positions.append((x, y))
        return obstacle_positions
    # 정지선까지 거리 실시간 계산 후 도달 판단.
    def calculate_distance_to_stopline(self):
        if self.current_pose is None or self.stopline_position is None:
            self.get_logger().warn("Missing current_pose or stopline_position")
            return None

        vehicle_x = self.current_pose.position.x
        vehicle_y = self.current_pose.position.y

        stopline_x = self.stopline_position.x
        stopline_y = self.stopline_position.y

        distance = np.hypot(stopline_x - vehicle_x, stopline_y - vehicle_y)

        self.get_logger().info(
            f"[DistanceCalc] Vehicle position: ({vehicle_x:.2f}, {vehicle_y:.2f}), "
            f"Stopline position: ({stopline_x:.2f}, {stopline_y:.2f}), "
            f"Distance: {distance:.2f}"
        )

        return distance

    
    # Frenet 시작 포인트
    def calc_nearest_s(self, x, y, s_step=0.1):
        s_values = np.arange(0, self.csp.s[-1], s_step)
        min_dist = float('inf')
        nearest_s = 0.0

        for s in s_values:
            ix, iy = self.csp.calc_position(s)
            dist = np.hypot(ix - x, iy - y)
            if dist < min_dist:
                min_dist = dist
                nearest_s = s

        return nearest_s
    def update_frenet_start_point(self):
        if self.current_pose is not None:
            position = self.current_pose.position
            # 차량 위치와 가장 가까운 s를 찾기 위한 방법
            s_nearest = self.calc_nearest_s(position.x, position.y)
            self.s0 = s_nearest
            self.get_logger().info(f'Frenet s0 updated: {self.s0:.2f}')

    # Frenet 초기화 관련
    # Global Path CSV 불러오기
    def load_global_path_from_csv(self, filepath):
        wx, wy = [], []
        with open(filepath, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                wx.append(float(row['x']))
                wy.append(float(row['y']))
        self.get_logger().info(f'Loaded {len(wx)} waypoints from CSV.')
        return wx, wy
    def init_frenet_planner(self):

        # CSV에서 waypoint 로딩
        csv_path = os.path.join(
        get_package_share_directory('local_planner'),
        'global_paths',
        'example_global_path.csv'
        )
        self.WX, self.WY = self.load_global_path_from_csv(csv_path)

        # 초기 상태 설정
        self.c_speed = 10.0 / 3.6  # 현재 속도[m/s]
        self.c_accel = 0.0         # 현재 가속도[m/s^2]
        self.c_d = 0.0             # 초기 lateral offset [m]
        self.c_d_d = 0.0           # lateral speed [m/s]
        self.c_d_dd = 0.0          # lateral accel [m/s^2]
        self.s0 = 0.0              # Frenet 경로 시작점 [m]

        # 장애물 리스트 (예시값)
        self.obstacles = np.array([[20.0, 10.0], [30.0, 6.0], [35.0, 8.0], [50.0, 3.0]])

        # Cubic spline planner 생성 (Global path)
        self.csp = cubic_spline_planner.CubicSpline2D(self.WX, self.WY)

        # Global path Marker 발행 추가
        self.publish_global_path()
        self.timer_global_path = self.create_timer(1.0, self.publish_global_path)

    # 시각화 관련 함수
    def publish_global_path(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # Global Path 두께 설정

        # Marker 색상 설정 (초록색 예시)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in zip(self.WX, self.WY):
            pt = Point(x=x, y=y, z=0.0)
            marker.points.append(pt)

        self.global_marker_publisher.publish(marker)
    def publish_stopline_limited_marker(self, path):
        distance_limit = self.calculate_distance_to_stopline()

        if path is None:
            self.get_logger().warn("[Pause] No path found to visualize.")
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stopline_limited_path"
        marker.id = 102
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0  # 노란색
        marker.color.b = 0.0
        marker.color.a = 1.0
        

        total_distance = 0.0
        last_x = self.current_pose.position.x
        last_y = self.current_pose.position.y

        for x, y in zip(path.x, path.y):
            segment_length = np.hypot(x - last_x, y - last_y)
            total_distance += segment_length
            last_x, last_y = x, y

            if distance_limit is not None and total_distance > distance_limit:
                break

            marker.points.append(Point(x=x, y=y, z=0.0))

        self.marker_publisher.publish(marker)
        if distance_limit is not None:
            self.get_logger().info(
            f"[Pause] Stopline-limited marker published. Distance limit: {distance_limit:.2f}"
        )
        else:
            self.get_logger().info(
            "[Pause] Stopline-limited marker published. Distance limit: ∞"
        )

    def is_near_obstacle(self, threshold=10.0):
        if self.obstacle_data is None or self.current_pose is None:
            return False
        vehicle_x = self.current_pose.position.x
        vehicle_y = self.current_pose.position.y
        for point in pc2.read_points(self.obstacle_data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if np.hypot(vehicle_x - x, vehicle_y - y) < threshold:
                return True
        return False

    def is_in_delivery_entry(self, threshold=10.0):
        if self.current_pose is None or not self.delivery_zone_polygon:
            return False
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        return self.point_in_polygon(pos[0], pos[1], self.delivery_zone_polygon)

    def is_in_delivery_center(self, threshold=20.0):
        return self.is_in_delivery_entry()

    def is_in_delivery_exit(self, threshold=15.0):
        # 기존대로 출구점 근처 원형 판정 유지(구역이 출구와 다를 수 있으므로)
        if self.current_pose is None:
            return False
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        return np.hypot(pos[0] - self.delivery_exit[0], pos[1] - self.delivery_exit[1]) < threshold

    def is_in_parking_entry(self, threshold=1.0):
        if self.current_pose is None or not self.closed_parking_zones:
            return False
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        for poly in self.closed_parking_zones:
            if self.point_in_polygon(pos[0], pos[1], poly):
                return True
        return False

    def is_on_open_parking_segment(self, threshold=1.0):
        if self.current_pose is None or not self.open_parking_segment:
            return False
        x0, y0 = self.current_pose.position.x, self.current_pose.position.y
        x1, y1 = self.open_parking_segment[0]
        x2, y2 = self.open_parking_segment[1]
        px = x2 - x1
        py = y2 - y1
        norm = px*px + py*py
        u = ((x0 - x1) * px + (y0 - y1) * py) / float(norm)
        u = max(0, min(1, u))
        x = x1 + u * px
        y = y1 + u * py
        dist = np.hypot(x0 - x, y0 - y)
        return dist < threshold

    def is_in_parking_center(self, threshold=20.0):
        return self.is_in_parking_entry()

    def is_in_parking_exit(self, threshold=20.0):
        if self.current_pose is None:
            return False
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        return np.hypot(pos[0] - self.parking_exit[0], pos[1] - self.parking_exit[1]) < threshold

    def is_in_unload_entry(self, threshold=15.0):
        if self.current_pose is None or not self.unload_zone_polygon:
            return False
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        return self.point_in_polygon(pos[0], pos[1], self.unload_zone_polygon)

    def is_in_unload_center(self, threshold=20.0):
        return self.is_in_unload_entry()

    def is_in_unload_exit(self, threshold=20.0):
        if self.current_pose is None:
            return False
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        return np.hypot(pos[0] - self.unload_exit[0], pos[1] - self.unload_exit[1]) < threshold

    # 주기적으로 실행되는 함수
    # 모드 실행함수 
    def publish_local_path(self):
        # --- 배달 미션 상태머신 ---
        if not self.delivery_zone_done:
            if self.delivery_state == 'NONE':
                if self.is_in_delivery_entry():
                    self.delivery_state = 'ENTRY'
                    self.get_logger().info('[Delivery] ENTRY (triggered by entry proximity)')
                    self.mission_state_pub.publish(String(data='DELIVERY_ENTRY'))
                    self.current_mode = ModeState.DELIVERY
            elif self.delivery_state == 'ENTRY':
                if self.is_in_delivery_center():
                    self.delivery_state = 'WAIT'
                    self.delivery_wait_timer = 0.0
                    self.get_logger().info('[Delivery] WAIT (5s)')
                    self.mission_state_pub.publish(String(data='DELIVERY_WAIT'))
            elif self.delivery_state == 'WAIT':
                self.delivery_wait_timer += 1.0
                if self.delivery_wait_timer >= 5.0:
                    self.delivery_state = 'EXIT'
                    self.get_logger().info('[Delivery] EXIT')
                    self.mission_state_pub.publish(String(data='DELIVERY_EXIT'))
                    self.current_mode = ModeState.DRIVE
            elif self.delivery_state == 'EXIT':
                if self.is_in_delivery_exit():
                    self.delivery_state = 'DONE'
                    self.delivery_zone_done = True
                    self.get_logger().info('[Delivery] DONE')
                    self.mission_state_pub.publish(String(data='DELIVERY_DONE'))
                    self.current_mode = ModeState.DRIVE
        # --- 주차 미션 상태머신 ---
        if not self.parking_zone_done:
            if self.parking_state == 'NONE':
                if self.is_in_parking_entry():
                    self.parking_state = 'ENTRY'
                    self.get_logger().info('[Parking] ENTRY (triggered by entry proximity)')
                    self.mission_state_pub.publish(String(data='PARKING_ENTRY'))
                    self.current_mode = ModeState.PARKING
            elif self.parking_state == 'ENTRY':
                if self.is_on_open_parking_segment():
                    self.parking_state = 'WAIT'
                    self.parking_wait_timer = 0.0
                    self.get_logger().info('[Parking] WAIT (5s)')
                    self.mission_state_pub.publish(String(data='PARKING_WAIT'))
            elif self.parking_state == 'WAIT':
                self.parking_wait_timer += 1.0
                if self.parking_wait_timer >= 5.0:
                    self.parking_state = 'EXIT'
                    self.get_logger().info('[Parking] EXIT')
                    self.mission_state_pub.publish(String(data='PARKING_EXIT'))
                    self.current_mode = ModeState.DRIVE
            elif self.parking_state == 'EXIT':
                if self.is_in_parking_exit():
                    self.parking_state = 'DONE'
                    self.parking_zone_done = True
                    self.get_logger().info('[Parking] DONE')
                    self.mission_state_pub.publish(String(data='PARKING_DONE'))
                    self.current_mode = ModeState.DRIVE
        # --- 정지선 미션 상태머신 (좌표 기반 ID) ---
        if self.stopline_centers:
            for center in self.stopline_centers:
                id = (round(center[0], 2), round(center[1], 2))
                if not self.stopline_flags.get(id, False):
                    if self.current_pose is not None:
                        pos = (self.current_pose.position.x, self.current_pose.position.y)
                        dist = np.hypot(pos[0] - center[0], pos[1] - center[1])
                        self.get_logger().info(f"[Stopline] Car to stopline {center}: {dist:.2f}m")
                        if dist < 10.0:
                            self.stopline_state = 'WAIT'
                            self.current_stopline_id = id
                            self.stopline_wait_timer = 0.0
                            self.get_logger().info(f'[Stopline] WAIT at stopline {id}')
                            self.mission_state_pub.publish(String(data=f'STOPLINE_WAIT_{id}'))
                            self.current_mode = ModeState.PAUSE
                elif self.stopline_state == 'WAIT' and self.current_stopline_id == id:
                    self.stopline_wait_timer += 1.0
                    if self.stopline_wait_timer >= 5.0:
                        self.stopline_flags[id] = True
                        self.stopline_state = 'DONE'
                        self.get_logger().info(f'[Stopline] DONE at stopline {id}')
                        self.mission_state_pub.publish(String(data=f'STOPLINE_DONE_{id}'))
                        self.current_mode = ModeState.DRIVE
        # --- 장애물 미션 상태머신 (좌표 기반 ID) ---
        if self.obstacle_data is not None:
            for x, y, z in pc2.read_points(self.obstacle_data, field_names=("x", "y", "z"), skip_nans=True):
                id = (round(x, 2), round(y, 2))
                if not self.obstacle_flags.get(id, False):
                    if self.current_pose is not None:
                        pos = (self.current_pose.position.x, self.current_pose.position.y)
                        if np.hypot(pos[0] - x, pos[1] - y) < 10.0:
                            self.obstacle_flags[id] = True
                            self.get_logger().info(f'[Obstacle] AVOID/DO at obstacle {id}')
                            self.mission_state_pub.publish(String(data=f'OBSTACLE_DONE_{id}'))
                            # 장애물 감지 시 OBSTACLE_STATIC 모드로 변경 (수동 트리거 가정)
                            # 실제는 장애물 종류, 위치 등 복합적으로 판단하여 모드 변경 필요
                            # 여기서는 단순히 감지 시 모드 변경 예시
                            self.current_mode = ModeState.OBSTACLE_STATIC
        # --- 하차 미션 상태머신 추가 ---
        if not self.unload_zone_done:
            if self.unload_state == 'NONE':
                # is_in_unload_entry 함수 필요 (local_planner_node에 이미 정의되어 있어야 함)
                if self.is_in_unload_entry():
                    self.unload_state = 'ENTRY'
                    self.get_logger().info('[Unload] ENTRY (triggered by entry proximity)')
                    self.mission_state_pub.publish(String(data='UNLOAD_ENTRY'))
                    self.current_mode = ModeState.DELIVERY # 하차도 DELIVERY 모드 사용 가정
            elif self.unload_state == 'ENTRY':
                 # is_in_unload_center 함수 필요
                if self.is_in_unload_center():
                    self.unload_state = 'WAIT'
                    self.unload_wait_timer = 0.0
                    self.get_logger().info('[Unload] WAIT (5s)')
                    self.mission_state_pub.publish(String(data='UNLOAD_WAIT'))
            elif self.unload_state == 'WAIT':
                self.unload_wait_timer += 1.0
                if self.unload_wait_timer >= 5.0:
                    self.unload_state = 'EXIT'
                    self.get_logger().info('[Unload] EXIT')
                    self.mission_state_pub.publish(String(data='UNLOAD_EXIT'))
            elif self.unload_state == 'EXIT':
                 # is_in_unload_exit 함수 필요
                if self.is_in_unload_exit():
                    self.unload_state = 'DONE'
                    self.unload_zone_done = True
                    self.get_logger().info('[Unload] DONE')
                    self.mission_state_pub.publish(String(data='UNLOAD_DONE'))
                    self.current_mode = ModeState.DRIVE # 미션 완료 후 DRIVE 모드로 복귀
        # --- 기존 모드별 경로 생성 ---
        if self.current_mode == ModeState.DRIVE:
            self.run_drive_mode()
        elif self.current_mode == ModeState.PAUSE:
            # 정지선 상태에 따라 경로 생성
            if self.stopline_state == 'WAIT' and self.current_stopline_id is not None:
                # 정지선까지의 경로만 생성
                stopline_idx = None
                for i, center in enumerate(self.stopline_centers):
                    if (round(center[0], 2), round(center[1], 2)) == self.current_stopline_id:
                        stopline_idx = i
                        break
                if stopline_idx is not None:
                    # 정지선까지의 경로만 생성하는 함수 필요
                    self.run_pause_mode()
                else:
                    self.run_drive_mode()
            else:
                self.run_drive_mode()
        elif self.current_mode == ModeState.OBSTACLE_STATIC:
            # 장애물 회피 상태에 따라 경로 생성
            if self.obstacle_state == 'AVOID':
                # 회피 경로 생성 (장애물 회피 경로)
                self.run_static_obstacle_mode()
            elif self.obstacle_state == 'RECOVER':
                # 복귀점으로 Hybrid A* 경로 생성
                self.goal_pose = Pose()
                self.goal_pose.position.x = float(self.obstacle_avoid_recover[0])
                self.goal_pose.position.y = float(self.obstacle_avoid_recover[1])
                self.run_delivery_mode()
            else:
                self.run_drive_mode()
        elif self.current_mode == ModeState.DELIVERY:
            # 배달 미션 상태에 따라 경로 생성
            if self.delivery_state in ['ENTRY', 'WAIT']:
                # 배달구역 중심으로 Hybrid A* 경로 생성
                self.goal_pose = Pose()
                self.goal_pose.position.x = float(self.delivery_center[0])
                self.goal_pose.position.y = float(self.delivery_center[1])
                self.run_delivery_mode()
            elif self.delivery_state == 'EXIT':
                # 복귀점으로 Hybrid A* 경로 생성
                self.goal_pose = Pose()
                self.goal_pose.position.x = float(self.delivery_exit[0])
                self.goal_pose.position.y = float(self.delivery_exit[1])
                self.run_delivery_mode()
            # --- 하차 미션일 경우 --- #
            elif self.unload_state in ['ENTRY', 'WAIT']:
                # 하차구역 중심으로 Hybrid A* 경로 생성
                self.goal_pose = Pose()
                self.goal_pose.position.x = float(self.unload_center[0])
                self.goal_pose.position.y = float(self.unload_center[1])
                self.run_delivery_mode()
            elif self.unload_state == 'EXIT':
                # 복귀점으로 Hybrid A* 경로 생성
                self.goal_pose = Pose()
                self.goal_pose.position.x = float(self.unload_exit[0])
                self.goal_pose.position.y = float(self.unload_exit[1])
                self.run_delivery_mode()
            else:
                self.run_drive_mode()
        elif self.current_mode == ModeState.PARKING:
            # 주차 미션 상태에 따라 경로 생성
            if self.parking_state in ['ENTRY', 'WAIT']:
                if self.last_generated_state != 'PARKING_SHA':
                    self.run_parking_sha_star()
                    self.last_generated_state = 'PARKING_SHA'
            elif self.parking_state == 'EXIT':
                # 복귀점으로 Hybrid A* 경로 생성
                self.goal_pose = Pose()
                self.goal_pose.position.x = float(self.parking_exit[0])
                self.goal_pose.position.y = float(self.parking_exit[1])
                self.run_delivery_mode()
            else:
                self.run_drive_mode()
        else:
            self.get_logger().warn("Unhandled mode. No path generated.")

        # 현재 상태를 문자열로 구성하여 발행
        current_mission_state = "GLOBAL_CRUISE"
        if self.delivery_state != 'NONE':
            current_mission_state = f'DELIVERY_{self.delivery_state}'
        elif self.parking_state != 'NONE':
             current_mission_state = f'PARKING_{self.parking_state}'
        elif self.stopline_state != 'NONE':
             current_mission_state = f'STOPLINE_{self.stopline_state}'
        elif self.obstacle_state != 'NONE':
             current_mission_state = f'OBSTACLE_{self.obstacle_state}'
        elif self.unload_state != 'NONE':
             current_mission_state = f'UNLOAD_{self.unload_state}'

        # 미션 상태 퍼블리시
        state_msg = String()
        state_msg.data = current_mission_state
        self.mission_state_pub.publish(state_msg)

        # 로컬 경로 생성 및 발행 (기존 코드 유지)
        # ... (기존 로컬 경로 발행 코드)

    # 주행 모드 
    def run_drive_mode(self):
        self.update_frenet_start_point()
        obstacle_positions = []
        if self.obstacle_data is not None:
            obstacle_positions = self.get_obstacle_positions(self.obstacle_data)

        if len(obstacle_positions) == 0:
            obstacles_np = np.empty((0, 2))
        else:
            obstacles_np = np.array(obstacle_positions)

        path, _ = fot.frenet_optimal_planning(
            self.csp,
            self.s0,
            self.c_speed,
            self.c_accel,
            self.c_d,
            self.c_d_d,
            self.c_d_dd,
            obstacles_np
        )

        # 닫힌 Polygon 위 경로 제한
        if path is not None:
            cut_idx = None
            for i, (x, y) in enumerate(zip(path.x, path.y)):
                for poly in self.closed_parking_zones:
                    if self.point_in_polygon(x, y, poly):
                        cut_idx = i
                        break
                if cut_idx is not None:
                    break
            if cut_idx is not None:
                if cut_idx > 1:
                    path.x = path.x[:cut_idx]
                    path.y = path.y[:cut_idx]
                else:
                    path = None

        # 경로가 없으면 마지막 유효 경로 유지 또는 글로벌 경로 fallback
        if path is not None:
            self.last_valid_path = path
        else:
            if self.last_valid_path is not None:
                path = self.last_valid_path
            else:
                # 글로벌 경로 fallback (간단히 앞 10개 점)
                if self.global_waypoints:
                    path = type('Path', (), {})()
                    path.x = [p[0] for p in self.global_waypoints[:10]]
                    path.y = [p[1] for p in self.global_waypoints[:10]]

        if path is not None:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.ns = "drive"
            marker.id = 102

            for x, y in zip(path.x, path.y):
                marker.points.append(Point(x=x, y=y, z=0.0))

            self.marker_publisher.publish(marker)
            self.get_logger().info(f'[Drive] Frenet path published ({len(obstacle_positions)} obstacles)')

            pose_array = PoseArray()
            pose_array.header.frame_id = 'drive'
            pose_array.header.stamp = self.get_clock().now().to_msg()
            for x, y in zip(path.x, path.y):
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose_array.poses.append(pose)
            self.path_publisher.publish(pose_array)
        else:
            self.get_logger().warn('[Drive] No path found')
    # 정지 모드
    def run_pause_mode(self):
        self.update_frenet_start_point()
        distance_to_stopline = self.calculate_distance_to_stopline()
        if distance_to_stopline is None:
            self.get_logger().warn("Stopline or localization unavailable")
            return
        if distance_to_stopline > 3.0:
            self.c_speed = min(self.c_speed, 3.0 / 3.6)
        else:
            if self.current_traffic_light == "RED":
                self.c_speed = 0.0
            elif self.current_traffic_light == "GREEN":
                self.c_speed = 10.0 / 3.6
        obstacles_np = np.empty((0, 2))
        path, _ = fot.frenet_optimal_planning(
            self.csp,
            self.s0,
            self.c_speed,
            self.c_accel,
            self.c_d,
            self.c_d_d,
            self.c_d_dd,
            obstacles_np
        )
        if path is not None:
            self.publish_stopline_limited_marker(path)
            self.get_logger().info(f"[Pause] Stopline dist: {distance_to_stopline:.2f}, traffic: {self.current_traffic_light}")
            pose_array = PoseArray()
            pose_array.header.frame_id = 'pause'
            pose_array.header.stamp = self.get_clock().now().to_msg()
            for x, y in zip(path.x, path.y):
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose_array.poses.append(pose)
            self.path_publisher.publish(pose_array)
        else:
            self.get_logger().warn("[Pause] No path generated")
    # 정적 장애물 모드
    def run_static_obstacle_mode(self):
        self.update_frenet_start_point()
        if self.obstacle_data is not None:
            obstacle_positions = self.get_obstacle_positions(self.obstacle_data)
        if len(obstacle_positions) == 0:
            self.get_logger().warn('[Static Obstacle] No obstacle received!')
            obstacles_np = np.empty((0, 2))
        else:
            obstacles_np = np.array(obstacle_positions)
        path, _ = fot.frenet_optimal_planning(
            self.csp,
            self.s0,
            self.c_speed,
            self.c_accel,
            self.c_d,
            self.c_d_d,
            self.c_d_dd,
            obstacles_np
        )
        if path is not None:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.25
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.ns = "static_obstacle"
            marker.id = 100
            for x, y in zip(path.x, path.y):
                marker.points.append(Point(x=x, y=y, z=0.0))
            self.marker_publisher.publish(marker)
            self.get_logger().info(f'[Static Obstacle] Path published with {len(obstacle_positions)} obstacles')
            pose_array = PoseArray()
            pose_array.header.frame_id = 'static_obstacle'
            pose_array.header.stamp = self.get_clock().now().to_msg()
            for x, y in zip(path.x, path.y):
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose_array.poses.append(pose)
            self.path_publisher.publish(pose_array)
        else:
            self.get_logger().warn('[Static Obstacle] No valid path found!')
    
    # Hybrid A* 경로 계획
    def run_delivery_mode(self):
        if self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn("[Delivery] Waiting for localization or goal pose.")
            return
        goal_pos = (self.goal_pose.position.x, self.goal_pose.position.y)
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        distance_to_goal = np.hypot(goal_pos[0] - current_pos[0], goal_pos[1] - current_pos[1])
        approach_threshold = 1.0
        if distance_to_goal < approach_threshold:
            self.get_logger().info(f"[Delivery] Reached goal ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}). Distance: {distance_to_goal:.2f}")
            return
        start = [self.current_pose.position.x, self.current_pose.position.y, self.get_yaw_from_quaternion(self.current_pose.orientation)]
        goal = [self.goal_pose.position.x, self.goal_pose.position.y, 0.0]
        self.get_logger().info(f"[Delivery] Hybrid A* planning from {start} to {goal}")
        obstacle_positions = self.get_obstacle_positions(self.obstacle_data) if self.obstacle_data else []
        ox = [p[0] for p in obstacle_positions]
        oy = [p[1] for p in obstacle_positions]
        # --- Polygon/Circle 구역을 Hybrid A* 장애물로 추가 ---
        for typ, param in self.obstacle_zones:
            if typ == 'polygon':
                poly = param
                for x, y in poly:
                    ox.append(x)
                    oy.append(y)
            elif typ == 'circle':
                cx, cy, r = param
                for i in range(16):
                    theta = 2 * np.pi * i / 16
                    ox.append(cx + r * np.cos(theta))
                    oy.append(cy + r * np.sin(theta))
        xy_resolution = 0.5
        yaw_resolution = np.deg2rad(15.0)
        path = hybrid_a_star_planning(start, goal, ox, oy, xy_resolution, yaw_resolution)
        if path and path.x_list:
            # --- 장애물 영역(구역) 충돌 체크 및 경로 자르기 ---
            cut_idx = None
            for i, (x, y) in enumerate(zip(path.x_list, path.y_list)):
                for poly in self.closed_parking_zones:
                    if self.point_in_polygon(x, y, poly):
                        cut_idx = i
                        break
                if cut_idx is not None:
                    break
            if cut_idx is not None:
                if cut_idx > 1:
                    path_x = path.x_list[:cut_idx]
                    path_y = path.y_list[:cut_idx]
                else:
                    path = None
            else:
                path_x = path.x_list
                path_y = path.y_list

            if path and path.x_list:
                self.get_logger().info(f"[Delivery] Hybrid A* generated path with {len(path_x)} points.")

                # 시각화를 위해 Marker 발행 (기존 코드 유지)
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "delivery"
                marker.id = 120
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.3
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0

                for x, y in zip(path_x, path_y):
                    marker.points.append(Point(x=x, y=y, z=0.0))

                self.marker_publisher.publish(marker)

                # 로컬 경로 발행 (기존 코드 유지)
                pose_array = PoseArray()
                pose_array.header.frame_id = 'delivery' # 프레임 ID를 'delivery'로 설정하여 PlotNode에서 구분
                pose_array.header.stamp = self.get_clock().now().to_msg()
                for x, y in zip(path_x, path_y):
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose_array.poses.append(pose)
                self.path_publisher.publish(pose_array)

                self.get_logger().info(f"[Delivery] Hybrid A* path published to goal ({goal[0]:.2f}, {goal[1]:.2f})")

        else:
            self.get_logger().warn("[Delivery] Hybrid A* failed to generate path.")

    def obstacle_zone_callback(self, msg: PoseArray):
        # PoseArray: 원형은 z에 반지름, 사각형은 z=0
        circles = []
        polygons = []
        # 원형: 연속된 2개 pose의 z>0이면 원, z==0이면 polygon vertex
        poses = msg.poses
        i = 0
        while i < len(poses):
            p = poses[i]
            if p.position.z > 0.0:
                # 원형 장애물
                circles.append(('circle', (p.position.x, p.position.y, p.position.z)))
                i += 1
            else:
                # polygon: z==0인 연속 pose들
                poly = []
                while i < len(poses) and poses[i].position.z == 0.0:
                    poly.append((poses[i].position.x, poses[i].position.y))
                    i += 1
                if len(poly) >= 3:
                    polygons.append(('polygon', poly))
        self.obstacle_zones = circles + polygons

    def is_collision_with_zones(self, x, y, margin=0.0):
        for typ, param in self.obstacle_zones:
            if typ == 'circle':
                cx, cy, r = param
                if r <= 0.0:
                    continue  # 반지름 0 이하는 무시
                if (x-cx)**2 + (y-cy)**2 <= (r+margin)**2:
                    return True
            elif typ == 'polygon':
                poly = param
                if self.point_in_polygon(x, y, poly):
                    return True
        return False

    def point_in_polygon(self, x, y, poly):
        # ray casting algorithm
        n = len(poly)
        inside = False
        px, py = x, y
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i+1)%n]
            if ((y1 > py) != (y2 > py)) and (px < (x2-x1)*(py-y1)/(y2-y1+1e-9)+x1):
                inside = not inside
        return inside

    def delivery_zone_callback(self, msg):
        self.delivery_zone_polygon = [(p.position.x, p.position.y) for p in msg.poses]
    def unload_zone_callback(self, msg):
        self.unload_zone_polygon = [(p.position.x, p.position.y) for p in msg.poses]
    def parking_zone_callback(self, msg):
        # PoseArray로 여러 Polygon(4개씩) 저장
        poses = msg.poses
        self.closed_parking_zones = []
        for i in range(0, len(poses), 4):
            poly = [(poses[i+j].position.x, poses[i+j].position.y) for j in range(4)]
            self.closed_parking_zones.append(poly)
    # 열린 영역(선분)은 별도 토픽/콜백에서 저장 필요

    def reset_missions(self):
        self.get_logger().info('[Mission] All mission states reset!')
        self.unload_state = 'NONE'
        self.unload_zone_done = False
        self.delivery_state = 'NONE'
        self.delivery_zone_done = False
        self.parking_state = 'NONE'
        self.parking_zone_done = False
        self.stopline_state = 'NONE'
        self.stopline_zone_done = False
        self.obstacle_state = 'NONE'
        self.obstacle_zone_done = False
        # 플래그 딕셔너리도 초기화
        self.stopline_flags = {}
        self.obstacle_flags = {}
        self.current_stopline_id = None
        self.current_obstacle_id = None
        self.get_logger().info('[Mission] All flags and states set to NONE/False.')

    def reset_missions_callback(self, msg):
        self.reset_missions()

    def run_parking_sha_star(self):
        if self.current_pose is None or not self.open_parking_segment:
            return
        # 시작점: 차량 현위치
        start = [
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.get_yaw_from_quaternion(self.current_pose.orientation)
        ]
        # 목표점: 열린 영역(선분) 중앙
        x1, y1 = self.open_parking_segment[0]
        x2, y2 = self.open_parking_segment[1]
        goal = [
            (x1 + x2) / 2,
            (y1 + y2) / 2,
            0.0
        ]
        # 장애물: 닫힌 Polygon을 조밀하게 샘플링해서 ox, oy에 추가
        ox, oy = [], []
        for poly in self.closed_parking_zones:
            for i in range(len(poly)):
                x0, y0 = poly[i]
                x1p, y1p = poly[(i+1)%len(poly)]
                for t in np.linspace(0, 1, 10):
                    ox.append(x0 + t*(x1p-x0))
                    oy.append(y0 + t*(y1p-y0))
        # 기타 장애물도 필요시 추가
        xy_resolution = 0.5
        yaw_resolution = np.deg2rad(15.0)
        path = hybrid_a_star_planning(start, goal, ox, oy, xy_resolution, yaw_resolution)
        # 경로가 닫힌 Polygon 침범 시 자르기/무효화
        if path and path.x_list:
            cut_idx = None
            for i, (x, y) in enumerate(zip(path.x_list, path.y_list)):
                for poly in self.closed_parking_zones:
                    if self.point_in_polygon(x, y, poly):
                        cut_idx = i
                        break
                if cut_idx is not None:
                    break
            if cut_idx is not None:
                if cut_idx > 1:
                    path_x = path.x_list[:cut_idx]
                    path_y = path.y_list[:cut_idx]
                else:
                    path = None
            else:
                path_x = path.x_list
                path_y = path.y_list
            if path and path.x_list:
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "parking_sha"
                marker.id = 130
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.3
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                for x, y in zip(path_x, path_y):
                    marker.points.append(Point(x=x, y=y, z=0.0))
                self.marker_publisher.publish(marker)
                pose_array = PoseArray()
                pose_array.header.frame_id = 'parking_sha'
                pose_array.header.stamp = self.get_clock().now().to_msg()
                for x, y in zip(path_x, path_y):
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose_array.poses.append(pose)
                self.path_publisher.publish(pose_array)
                self.get_logger().info(f"[Parking SHA*] Path published to open zone center ({goal[0]:.2f}, {goal[1]:.2f})")
            else:
                self.get_logger().warn("[Parking SHA*] No valid path found!")
        else:
            self.get_logger().warn("[Parking SHA*] Hybrid A* failed to generate path.")

def main(args=None):
    rclpy.init(args=args)
    local_planner_node = LocalPlanner()
    rclpy.spin(local_planner_node)
    local_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
