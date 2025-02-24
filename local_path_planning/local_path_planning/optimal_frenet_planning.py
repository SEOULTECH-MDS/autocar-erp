#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

# ROS
import rclpy
from rclpy.node import Node
import message_filters

from geometry_msgs.msg import PoseArray, Point, Vector3, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, ColorRGBA, Bool, String, Int8
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from autocar_msgs.msg import Path2D
from autocar_msgs.msg import State2D 


from autocar_nav.yaw_to_quaternion import yaw_to_quaternion
from autocar_nav.euler_from_quaternion import euler_from_quaternion

# Path Finder
from .path_finder.utils import generate_target_course
from .path_finder.frenet import Frenet
from .path_finder.parking import Parking
from .path_finder.delivery import Delivery

# =============================================
ROBOT_RADIUS = 1.0 # [m]
LANE_WIDTH = 3.6 # [m]

class OptimalFrenetPlanning(Node):
    def __init__(self):
        super().__init__('optimal_frenet_planning')

        self.driving_mode = 'normal_driving'
        self.gear_override = 0
        
        self.car_pose = None
        self.obstacles = []
        self.obstacles_hx = [] # obstacles history
        self.mission_pose = None

        self.ref_path = None
        self.possible_change_direction = None
        
        self.path_finder = None
        self.delivery_pose = None
    

        # Subscribers
        self.global_waypoints_sub = self.create_subscription(PoseArray, '/global_waypoints',  self.callback_near_ways, 10)
        self.location_sub = self.create_subscription(State2D, '/autocar/state2D', self.callback_local_path_planning, 10)

        # self.location_sub = self.create_subscription(Odometry, '/location_corrected',  self.callback_local_path_planning, 10)
        # self.obstacle_sub = self.create_subscription(PoseArray, '/obstacles_utm', self.callback_obstacles, 10)

        # self.delivery_pose_sub = self.create_subscription(PoseArray, '/delivery_utm', self.callback_delivery_pose, 10)
        # self.driving_mode_sub = self.create_subscription(String, '/driving_mode', self.callback_driving_mode, 10)
        # self.gear_override_sub = self.create_subscription(Int8, '/gear/override', self.callback_gear_override, 10)

        # Publishers
        self.local_path_pub = self.create_publisher(Path, '/autocar/path', 10)
        self.candidate_path_pub = self.create_publisher(Marker, "/local_candidate_paths", 10)
        self.is_avoiding_pub = self.create_publisher(Bool, "/is_avoiding", 10)
        self.gear_pub = self.create_publisher(Int8, '/gear', 10)

    
    def callback_local_path_planning(self, location_msg):
        # 현재 차량 위치
        car_x, car_y = location_msg.pose.x, location_msg.pose.y
        car_yaw = location_msg.pose.theta
        # car_x, car_y = location_msg.pose.pose.position.x, location_msg.pose.pose.position.y
        # car_yaw = euler_from_quaternion([location_msg.pose.pose.orientation.x, location_msg.pose.pose.orientation.y,\
        #                                  location_msg.pose.pose.orientation.z, location_msg.pose.pose.orientation.w])[2] 기존코드

        self.car_pose = [car_x, car_y, car_yaw]
        
        if self.path_finder is None:
            return
        
        candidate_paths, opt_path, is_avoiding, gear, _ = self.path_finder.find_path(self.car_pose, self.obstacles)

        # ROS Publish (1): Candidate paths
        all_candidate_paths = self.make_all_paths_msg(candidate_paths)
        self.candidate_path_pub.publish(all_candidate_paths)

        # ROS Publish (2): final local path
        path_msg = self.make_path_msg(opt_path)
        self.local_path_pub.publish(path_msg)

        # ROS Publish (3): avoidance status
        as_msg = Bool()
        as_msg.data = is_avoiding
        self.is_avoiding_pub.publish(as_msg)

        # ROS Publish (4): gear
        gear_msg = Int8()
        if self.gear_override == 1:
            gear = 1
        gear_msg.data = gear
        self.gear_pub.publish(gear_msg)
        
        print("현재 장애물 회피 중?:", is_avoiding)
        print("주행 모드: ", self.driving_mode)
        print("기어: ", gear)
        print("ref path wp 수", len(opt_path[0]))
        return
    
    def callback_near_ways(self, nodes_msg):
        if self.car_pose is None:
            print("Localization 키세요")
            return
        
        # 경로 정보
        self.ref_path, self.possible_change_direction = self.get_path(nodes_msg)

        self.path_finder = Frenet(self.ref_path, self.car_pose,
                                robot_radius = ROBOT_RADIUS,
                                lane_width = LANE_WIDTH,
                                possible_change_direction = self.possible_change_direction)
        return
    
    def callback_obstacles(self, obstacle_msg):
        obstacles = []
        for ob in obstacle_msg.poses:
            x,y = ob.position.x, ob.position.y
            radius = ob.position.z
            obstacles.append([x,y,radius])

        self.obstacles_hx.append(obstacles)
        # obstacles_hx의 길이가 4를 넘는 경우, 가장 오래된 항목 제거
        if len(self.obstacles_hx) > 4:
            excess_length = len(self.obstacles_hx) - 4
            self.obstacles_hx = self.obstacles_hx[excess_length:]

        self.obstacles = []
        for obs in self.obstacles_hx:
            self.obstacles += obs
        
        return
    
    def callback_delivery_pose(self, poses_msg):
        poses = []
        for pose in poses_msg.poses:
            x,y = pose.position.x,pose.position.y
            poses.append([x,y])
            
        self.delivery_pose = poses

    def callback_driving_mode(self, mode_msg):
        if self.ref_path is None:
            return
        
        # 정상주행 => 배달미션(A)로 넘어갈 때
        if (self.driving_mode == 'normal_driving' or self.driving_mode == 'curve')  and mode_msg.data == 'delivery_start':
            if self.delivery_pose is None:
                print("아직 배달 표지판 못찾음,", self.delivery_pose)
                self.path_finder = Frenet(self.ref_path, self.car_pose,
                                robot_radius = ROBOT_RADIUS,
                                lane_width = LANE_WIDTH,
                                possible_change_direction = self.possible_change_direction)
            
            else:
                print("배달A 모드로 전환")
                self.path_finder = Delivery(self.ref_path, self.car_pose, self.delivery_pose,
                                            min_R = 7.0, 
                                            delivery_mode = 'delivery_A')
                self.driving_mode = mode_msg.data
            

        # 배달미션(A) => 정상주행으로 넘어갈 때
        elif self.driving_mode == 'delivery_start' and (mode_msg.data == 'normal_driving' or\
                                                 mode_msg.data == 'curve'):
            print("정상 주행 모드로 전환")
            self.path_finder = Frenet(self.ref_path, self.car_pose,
                                robot_radius = ROBOT_RADIUS,
                                lane_width = LANE_WIDTH,
                                possible_change_direction = self.possible_change_direction)
            self.driving_mode = mode_msg.data
            self.delivery_pose = None

        # 정상주행 => 배달미션(B)로 넘어갈 때
        if (self.driving_mode == 'normal_driving' or self.driving_mode == 'curve') and mode_msg.data == 'delivery_finish':
            if self.delivery_pose is None:
                print("아직 배달 표지판 못찾음,", self.delivery_pose)
                self.path_finder = Frenet(self.ref_path, self.car_pose,
                                robot_radius = ROBOT_RADIUS,
                                lane_width = LANE_WIDTH,
                                possible_change_direction = self.possible_change_direction)
            else:
                print("배달B 모드로 전환")
                # 일단은 delivery_A라고 해놓음
                self.path_finder = Delivery(self.ref_path, self.car_pose, self.delivery_pose,
                                        min_R = 4.0,
                                        delivery_mode = 'delivery_B')
                self.driving_mode = mode_msg.data

        # 배달미션(B) => 정상주행으로 넘어갈 때
        elif self.driving_mode == 'delivery_finish' and (mode_msg.data == 'normal_driving' or\
                                                 mode_msg.data == 'curve'):
            print("정상 주행 모드로 전환")
            self.path_finder = Frenet(self.ref_path, self.car_pose,
                                robot_radius = ROBOT_RADIUS,
                                lane_width = LANE_WIDTH,
                                possible_change_direction = self.possible_change_direction)
            self.driving_mode = mode_msg.data

        # 정상 주행 => 주차 미션으로 넘어갈 때
        elif (self.driving_mode == 'normal_driving' or self.driving_mode == 'curve') and mode_msg.data == 'parking':
            print("주차 모드로 전환")
            self.path_finder = Parking(self.ref_path, self.car_pose,
                                    min_R = 4.0)
            self.driving_mode = mode_msg.data

        # 주차 미션 => 정상 주행으로 넘어갈 때
        elif self.driving_mode == 'parking' and (mode_msg.data == 'normal_driving' or\
                                                 mode_msg.data == 'curve'):
            print("정상 주행 모드로 전환")
            self.path_finder = Frenet(self.ref_path, self.car_pose,
                                robot_radius = ROBOT_RADIUS,
                                lane_width = LANE_WIDTH,
                                possible_change_direction = self.possible_change_direction)
            self.driving_mode = mode_msg.data

        return
    
    def callback_gear_override(self, gear_ov_msg):
        self.gear_override = gear_ov_msg.data

    def get_path(self, path_msg):
        possible_change_direction = path_msg.header.frame_id

        path = {'x': None, 'y': None, 'yaw': None, 's':None, 'csp':None}

        xs, ys = [], []
        for node in path_msg.poses:
            xs.append(node.position.x)
            ys.append(node.position.y)

        xs, ys, yaws, s, csp = generate_target_course(xs, ys, step_size=0.2)
        path['x'] = xs
        path['y'] = ys
        path['yaw'] = yaws
        path['s'] = s
        path['csp'] = csp

        return path, possible_change_direction
    
    def make_path_msg(self, path):
        
        path_x, path_y, path_yaw = path

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
    
    # def make_all_paths_msg(self, paths):
    #     paths_msg = Marker(
    #         header=Header(frame_id='world', stamp=self.get_clock().now().to_msg()),
    #         ns="candidate_paths",
    #         id=0,
    #         type=Marker.SPHERE_LIST,
    #         action=Marker.ADD,
    #         scale=Vector3(x=0.05, y=0.05, z=0.05),  # 구체 크기
    #         color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  
    #         lifetime=Duration(sec=0, nanosec=0)
    #     )

    #     for pth in paths:
    #         candidate_path = self.make_points(pth.x, pth.y)
    #         paths_msg.points.extend(candidate_path)

    #     return paths_msg

    def make_all_paths_msg(self, paths):
        paths_msg = Marker(
            header=Header(frame_id='world', stamp=self.get_clock().now().to_msg()),
            ns="candidate_paths",
            id=0,
            type=Marker.LINE_LIST,  # 구체 대신 연속적인 선으로 변경
            action=Marker.ADD,
            scale=Vector3(x=0.03, y=0.0, z=0.0),  # 선 두께 (x만 사용, y와 z는 무시됨)
            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # 초록색
            lifetime=Duration(sec=0, nanosec=0)  # 영구 지속
        )

        for pth in paths:
            candidate_path = self.make_points(pth.x, pth.y)
            paths_msg.points.extend(candidate_path)

        return paths_msg
    
    def make_points(self, path_x, path_y):
        points = []

        for i in range(len(path_x)):
            point = Point()
            point.x = path_x[i]
            point.y = path_y[i]
            point.z = 0.0
            points.append(point)

        return points
    
def main(args=None):
    # Initialise the node
    rclpy.init(args=args)
    
    try:
        # Initialise the class
        optimal_frenet_planning = OptimalFrenetPlanning()

        # Stop the node from exiting
        rclpy.spin(optimal_frenet_planning)

    finally:
        optimal_frenet_planning.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
