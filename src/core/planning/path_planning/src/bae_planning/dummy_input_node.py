#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, PoseArray
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from geometry_msgs.msg import PoseStamped
import csv
import os
from ament_index_python.packages import get_package_share_directory
import math
import numpy as np

class DummyInputNode(Node):
    def __init__(self):
        super().__init__('dummy_input_node')

        # Publishers
        self.stopline_pub = self.create_publisher(Point, '/stopline_detector', 10)
        self.traffic_pub = self.create_publisher(String, '/traffic_detector', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacle_detector', 10)
        self.stopline_dist_pub = self.create_publisher(Float32, '/stop_line_distance', 10)
        self.obstacle_bool_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.odom_pub = self.create_publisher(Odometry, '/localization', 10)
        self.global_path_pub = self.create_publisher(Path, '/global_waypoints', 10)
        self.delivery_sign_pub = self.create_publisher(String, '/sign_detector', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # --- 구역/정지선 PoseArray 퍼블리셔 ---
        self.delivery_zone_pub = self.create_publisher(PoseArray, '/delivery_zone', 10)
        self.unload_zone_pub = self.create_publisher(PoseArray, '/unload_zone', 10)
        self.parking_zone_pub = self.create_publisher(PoseArray, '/parking_zone', 10)
        self.stopline_array_pub = self.create_publisher(PoseArray, '/stopline_array', 10)
        # --- 장애물 구역 퍼블리셔 추가 ---
        self.obstacle_zone_pub = self.create_publisher(PoseArray, '/obstacle_zone', 10)

        # Subscribers
        self.create_subscription(PoseArray, '/local_path', self.local_path_callback, 10)

        # Variables
        self.sim_path_points = []
        self.sim_current_path_index = 0
        self.scenario_state = 'GLOBAL_CRUISE'
        self.state_timer = 0.0
        self.current_pos = (0.0, 0.0)
        # --- 차량 시뮬레이션 설정 ---
        self.timer_period = 0.1  # 타이머 주기 (초)
        self.base_sim_speed = 2.0 # 기본 시뮬레이션 속도 (m/s)
        self.sim_speed_multiplier = 0.7 # 속도 조절 (70%)
        self.current_sim_speed = self.base_sim_speed * self.sim_speed_multiplier
        self.global_path_index = 0 # 글로벌 경로 추종 인덱스
        # --- 배달 상차 구역 정보 ---
        self.delivery_pickup_zone = [(0, 90), (20, 90), (20, 120), (0, 120)]
        self.delivery_pickup_goal = (10, 105)  # 구역 중심
        self.wait_timer = 0.0
        # 하차 구역 정보
        self.unload_zone_vertices = [(0, 260), (20, 260), (20, 290), (0, 290)]
        self.unload_zone_center = (10, 275)
        self.unload_zone_entry = (10, 255)  # 진입 목표점
        self.unload_zone_exit = (10, 295)   # 복귀 목표점
        self.unload_wait_timer = 0.0
        self.unload_wait_duration = 5.0
        self.in_unload_zone = False
        self.unload_zone_done = False
        # --- 주차구역 정보 ---
        # 닫힌 영역1 (아래)
        closed_zone1 = [
            [(210, 230), (190, 230), (190, 245), (210, 245)],
            [(210, 245), (190, 245), (190, 260), (210, 260)]
        ]
        # 닫힌 영역2 (위)
        closed_zone2 = [
            [(210, 290), (190, 290), (190, 300), (210, 300)],
            [(210, 300), (190, 300), (190, 305), (210, 305)],
            [(210, 305), (190, 305), (190, 320), (210, 320)]
        ]
        self.closed_parking_zones = closed_zone1 + closed_zone2
        # 열린 영역(선분)
        self.open_parking_segment = [(210, 260), (210, 290)]
        self.parking_zone_center = (200, 275)
        self.parking_zone_entry = (200, 225)
        self.parking_zone_exit = (200, 325)
        self.parking_wait_timer = 0.0
        self.parking_wait_duration = 5.0
        self.in_parking_zone = False
        self.parking_zone_done = False
        # --- 정지선 정보 ---
        self.stopline_centers = [
            (0, 340), (90, 350), (30, 30), (130, 100), (190, 120)
        ]
        # --- 정지선 정지 플래그 ---
        self.stopline_wait_flags = [False] * len(self.stopline_centers)
        self.current_stopline_idx = None
        self.stopline_waiting = False
        self.stopline_wait_timer = 0.0
        self.stopline_last_point = None

        # Load global path
        csv_path = os.path.join(
            get_package_share_directory('local_planner'),
            'global_paths',
            'example_global_path.csv'
        )
        self.global_waypoints = self.load_global_path_from_csv(csv_path)

        # Publish global path
        self.publish_global_path()

        # Timer
        self.create_timer(0.1, self.scenario_timer_callback)

        # --- 장애물 구역 정보(세로 땅콩형, y축 겹침) ---
        self.circle_obstacles = [
            {'center': (40, 340), 'radius': 0.0},
            {'center': (40, 340), 'radius': 0.0},
            {'center': (60, 360), 'radius':0.0},
            {'center': (60, 360), 'radius':0.0},
        ]
        self.square_obstacles = [
            {'vertices': [(100-10,240-10), (100+10,240-10), (100+10,240+10), (100-10,240+10)]},
            {'vertices': [(100-10,190-10), (100+10,190-10), (100+10,190+10), (100-10,190+10)]},
        ]

    def load_global_path_from_csv(self, filepath):
        waypoints = []
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                waypoints.append((x, y))
        return waypoints

    def publish_global_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for x, y in self.global_waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.global_path_pub.publish(path_msg)
        self.get_logger().info(f"Published global path with {len(self.global_waypoints)} waypoints.")

    def local_path_callback(self, msg):
        # 새로운 로컬 경로를 받으면 저장하고 인덱스 및 글로벌 경로 추종 중지
        self.sim_path_points = [(pose.position.x, pose.position.y) for pose in msg.poses]
        # Find the closest point on the new local path to the current car position
        min_dist = float('inf')
        nearest_idx = 0
        if self.sim_path_points and self.current_pos:
            for i, (x, y) in enumerate(self.sim_path_points):
                dist = np.hypot(self.current_pos[0] - x, self.current_pos[1] - y)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            self.sim_current_path_index = nearest_idx
        else:
            self.sim_current_path_index = 0 # Fallback to 0 if no path or pos

        # When a local path is received, stop following the global path
        # The car will follow the local path, and transition back to global path logic 
        # when the local path is exhausted in the timer callback.
        self.get_logger().info(f"Received new local path for simulation with {len(self.sim_path_points)} points.")

    def scenario_timer_callback(self):
        # --- 차량 위치 시뮬레이션 ---
        if self.stopline_waiting:
            # 정지선 대기 중이면 차량 위치를 정지선 앞에 고정
            if self.stopline_last_point is not None:
                self.current_pos = self.stopline_last_point
            self.stopline_wait_timer += self.timer_period
            if self.stopline_wait_timer >= 5.0:
                self.stopline_waiting = False
                self.stopline_wait_timer = 0.0
                self.stopline_last_point = None
            # 이후 퍼블리시 및 미션 로직은 그대로 진행
        else:
            if self.sim_path_points:
                # 마지막 점이 정지선 근처인지 확인
                last_point = self.sim_path_points[-1]
                for stopline in self.stopline_centers:
                    if np.hypot(last_point[0] - stopline[0], last_point[1] - stopline[1]) < 10.0:
                        self.stopline_waiting = True
                        self.stopline_wait_timer = 0.0
                        self.stopline_last_point = last_point
                        self.current_pos = last_point
                        break
                else:
                    # 정지선 근처가 아니면 기존 로직대로 차량 위치 업데이트
                    if self.sim_current_path_index < len(self.sim_path_points):
                        current_point = self.sim_path_points[self.sim_current_path_index]
                        if self.sim_current_path_index < len(self.sim_path_points) - 1:
                            next_point = self.sim_path_points[self.sim_current_path_index + 1]
                            dist_to_next = np.hypot(next_point[0] - current_point[0], next_point[1] - current_point[1])
                            distance_to_move = self.current_sim_speed * self.timer_period
                            moved_distance = 0
                            steps_advanced = 0
                            current_idx = self.sim_current_path_index
                            while current_idx < len(self.sim_path_points) - 1:
                                p1 = self.sim_path_points[current_idx]
                                p2 = self.sim_path_points[current_idx + 1]
                                segment_dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
                                if moved_distance + segment_dist <= distance_to_move:
                                    moved_distance += segment_dist
                                    steps_advanced += 1
                                    current_idx += 1
                                else:
                                    break
                            self.sim_current_path_index += max(1, steps_advanced)
                        else:
                            self.sim_current_path_index = len(self.sim_path_points) - 1
                    if self.sim_current_path_index < len(self.sim_path_points):
                        self.current_pos = self.sim_path_points[self.sim_current_path_index]
                    else:
                        self.current_pos = self.sim_path_points[-1]
            else:
                # 로컬 경로가 없으면 글로벌 경로를 따라 이동 (기존 로직)
                if self.global_waypoints:
                    nearest_idx = self.find_nearest_global_idx(self.current_pos)
                    distance_to_move = self.current_sim_speed * self.timer_period
                    moved_distance = 0
                    current_idx = nearest_idx
                    while current_idx < len(self.global_waypoints) - 1:
                        p1 = self.global_waypoints[current_idx]
                        p2 = self.global_waypoints[current_idx + 1]
                        segment_dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
                        if moved_distance + segment_dist <= distance_to_move:
                            moved_distance += segment_dist
                            current_idx += 1
                        else:
                            break
                    self.global_path_index = current_idx
                    self.current_pos = self.global_waypoints[self.global_path_index]
                else:
                    pass

        # Odometry 퍼블리시
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = self.current_pos[0]
        odom.pose.pose.position.y = self.current_pos[1]
        # TODO: 차량 방향(orientation) 시뮬레이션 필요
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # --- 미션 상태 기반 시나리오 진행 ---
        # TODO: Add state machine logic for DELIVERY, PARKING, STOPLINE, OBSTACLE

        # --- 배달 상차 미션 --- (Example - Integrate into state machine below)
        # Current simplified state checks - replace with full state machine
        # For now, let local planner trigger, dummy just follows path

        # --- 정지선 퍼블리시 ---
        for center in self.stopline_centers:
            pt = Point()
            pt.x, pt.y, pt.z = float(center[0]), float(center[1]), 0.0
            self.stopline_pub.publish(pt)

        # --- 장애물 퍼블리시 ---
        obstacle_points = self.generate_obstacle_points()
        self.publish_obstacle_pointcloud(obstacle_points)
        # --- 장애물 구역(PoseArray) 퍼블리시 ---
        pa = PoseArray()
        pa.header.frame_id = 'map'
        # 원형 장애물: 중심점만 Pose로 추가, radius는 position.z에 저장
        for obs in self.circle_obstacles:
            if obs['radius'] <= 0.0:
                continue  # 반지름 0 이하는 퍼블리시하지 않음
            p = Pose()
            p.position.x = float(obs['center'][0])
            p.position.y = float(obs['center'][1])
            p.position.z = float(obs['radius'])  # z에 반지름 저장
            pa.poses.append(p)
        # 사각형 장애물: 각 꼭짓점을 Pose로 추가, z=0
        for obs in self.square_obstacles:
            for vx, vy in obs['vertices']:
                p = Pose()
                p.position.x = float(vx)
                p.position.y = float(vy)
                p.position.z = 0.0
                pa.poses.append(p)
        self.obstacle_zone_pub.publish(pa)

        # --- 배달구역 꼭짓점 퍼블리시 ---
        pa = PoseArray()
        pa.header.frame_id = 'map'
        for x, y in self.delivery_pickup_zone:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            pa.poses.append(p)
        self.delivery_zone_pub.publish(pa)

        # --- 하차구역 꼭짓점 퍼블리시 ---
        pa = PoseArray()
        pa.header.frame_id = 'map'
        for x, y in self.unload_zone_vertices:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            pa.poses.append(p)
        self.unload_zone_pub.publish(pa)

        # --- 주차구역 꼭짓점 퍼블리시 (닫힌 영역) ---
        pa = PoseArray()
        pa.header.frame_id = 'map'
        for poly in self.closed_parking_zones:
            for x, y in poly:
                p = Pose()
                p.position.x = float(x)
                p.position.y = float(y)
                pa.poses.append(p)
        self.parking_zone_pub.publish(pa)

        # --- 주차구역 열린 영역(선분) 퍼블리시 ---
        pa_open = PoseArray()
        pa_open.header.frame_id = 'map'
        for x, y in self.open_parking_segment:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            pa_open.poses.append(p)
        # 필요시 별도 토픽으로 퍼블리시 (예: self.open_parking_pub.publish(pa_open))

        # --- 정지선 중심들 퍼블리시 ---
        pa = PoseArray()
        pa.header.frame_id = 'map'
        for x, y in self.stopline_centers:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            pa.poses.append(p)
        self.stopline_array_pub.publish(pa)
        # 이후 다른 시나리오 상태 추가 가능

    def is_in_zone(self, pos, zone_points):
        xs, ys = zip(*zone_points)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        x, y = pos
        return (min_x <= x <= max_x) and (min_y <= y <= max_y)

    def distance(self, p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5

    def publish_goal_pose(self, goal_xy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goal_xy[0])
        goal.pose.position.y = float(goal_xy[1])
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published goal pose: ({goal_xy[0]}, {goal_xy[1]})")

    def find_nearest_global_idx(self, pos):
        min_dist = float('inf')
        min_idx = 0
        if self.global_waypoints:
            for i, (x, y) in enumerate(self.global_waypoints):
                dist = np.hypot(pos[0] - x, pos[1] - y)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i
        return min_idx

    def is_on_global_path(self, pos, threshold=2.0):
        return self.distance(pos, self.global_waypoints[self.find_nearest_global_idx(pos)]) < threshold

    def publish_goal_point(self, point):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = point[0]
        goal.pose.position.y = point[1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published goal point: ({point[0]}, {point[1]})")

    def is_near_point(self, pose, point, threshold=2.0):
        return self.distance(pose, point) < threshold

    def generate_obstacle_points(self):
        points = []
        # 원형 장애물 4개
        circle_centers = [(40, 346), (40, 342), (60, 359), (60, 355)]
        radius = 4.0
        for cx, cy in circle_centers:
            for i in range(16):
                theta = 2 * math.pi * i / 16
                x = cx + radius * math.cos(theta)
                y = cy + radius * math.sin(theta)
                points.append((x, y, 0.0))
        # 정사각형 장애물 2개
        square_centers = [(100, 240), (100, 190)]
        half = 10.0
        for cx, cy in square_centers:
            # 4변, 각 변 5점씩
            for i in range(5):
                t = i / 4
                # 아래변
                points.append((cx - half + 20*t, cy - half, 0.0))
                # 위변
                points.append((cx - half + 20*t, cy + half, 0.0))
                # 좌변
                points.append((cx - half, cy - half + 20*t, 0.0))
                # 우변
                points.append((cx + half, cy - half + 20*t, 0.0))
        return points

    def publish_obstacle_pointcloud(self, points):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)
        ]
        pc2_msg = pc2.create_cloud(header, fields, points)
        self.obstacle_pub.publish(pc2_msg)
        self.get_logger().debug(f"Published {len(points)} obstacle points as PointCloud2.")

def main(args=None):
    rclpy.init(args=args)
    node = DummyInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()