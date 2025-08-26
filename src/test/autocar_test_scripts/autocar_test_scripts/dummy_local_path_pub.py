#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
import numpy as np
import math
from rclpy.qos import QoSProfile, DurabilityPolicy
from autocar_utils.euler_from_quaternion import euler_from_quaternion

class DummyLocalPathPublisher(Node):
    def __init__(self):
        super().__init__('dummy_local_path_publisher')
        
        # QoS 설정 - TRANSIENT_LOCAL로 늦게 구독하는 노드도 메시지를 받을 수 있도록 함
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Path publisher
        self.path_pub = self.create_publisher(Path, '/waypoints', qos_profile=qos_transient_local)
        
        # Map origin subscriber
        self.map_origin_sub = self.create_subscription(
            PointStamped, '/map/origin', self.map_origin_cb, qos_transient_local
        )
        
        # Vehicle location subscriber (차량 위치를 한 번만 받기 위함)
        self.localization_sub = self.create_subscription(
            Odometry, '/autocar/location', self.vehicle_state_cb, 10
        )
        
        # 변수 초기화
        self.map_origin_x = None
        self.map_origin_y = None
        self.path_published = False
        
        # 차량 상태 (최초 한 번만 받음)
        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_yaw = None
        self.vehicle_state_received = False
        
        # 2초마다 path 생성 시도 (map origin과 차량 위치 모두 수신될 때까지)
        self.timer = self.create_timer(2.0, self.publish_dummy_path)
        
        self.get_logger().info("Dummy Local Path Publisher 초기화 완료")

    def map_origin_cb(self, msg):
        """Map origin 콜백"""
        self.map_origin_x = msg.point.x
        self.map_origin_y = msg.point.y
        self.get_logger().info(f"Map origin 설정: ({self.map_origin_x:.2f}, {self.map_origin_y:.2f})")
    
    def vehicle_state_cb(self, msg):
        """차량 상태 콜백 (최초 한 번만 받음)"""
        if self.vehicle_state_received:
            return  # 이미 받았으면 무시
        
        if self.map_origin_x is None or self.map_origin_y is None:
            self.get_logger().warn("Map origin 정보가 아직 설정되지 않았습니다.")
            return
        
        # Map origin 기준 상대 좌표로 변환
        self.vehicle_x = msg.pose.pose.position.x - self.map_origin_x
        self.vehicle_y = msg.pose.pose.position.y - self.map_origin_y
        
        # 쿼터니언을 오일러 각으로 변환하여 yaw 추출
        q = msg.pose.pose.orientation
        self.vehicle_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        self.vehicle_state_received = True
        self.get_logger().info(f"차량 위치 수신: ({self.vehicle_x:.2f}, {self.vehicle_y:.2f}), yaw: {self.vehicle_yaw:.2f} rad")
        
        # 구독 중지 (한 번만 받으면 됨)
        self.destroy_subscription(self.localization_sub)
    
    def publish_dummy_path(self):
        """더미 local path 생성 및 퍼블리시"""
        if self.path_published:
            return
        
        # Map origin과 차량 위치 정보가 모두 수신되었는지 확인
        if self.map_origin_x is None or self.map_origin_y is None:
            self.get_logger().warn("Map origin 정보가 아직 수신되지 않았습니다. 대기 중...")
            return
        
        if not self.vehicle_state_received or self.vehicle_x is None:
            self.get_logger().warn("차량 위치 정보가 아직 수신되지 않았습니다. 대기 중...")
            return
            
        # Path 메시지 생성
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        waypoints = []
        
        # 차량 현재 위치와 방향을 기준으로 경로 생성
        # 1단계: 차량 진행방향 기준 오른쪽으로 2미터 떨어진 시작점 계산
        # 차량의 오른쪽 방향은 현재 yaw에서 -90도 (시계방향)
        right_direction = self.vehicle_yaw - math.pi/2
        start_x = self.vehicle_x + 2.0 * math.cos(right_direction)
        start_y = self.vehicle_y + 2.0 * math.sin(right_direction)

        # 2단계: 시작점에서 차량 진행방향으로 10미터까지 경로 생성 (0.5m 간격)
        forward_distance = 20.0
        waypoint_interval = 0.5  # 0.5미터 간격
        num_points = int(forward_distance / waypoint_interval)  # 10m / 0.5m = 20개 구간

        for i in range(num_points + 1):
            x = start_x + i * waypoint_interval * math.cos(self.vehicle_yaw)
            y = start_y + i * waypoint_interval * math.sin(self.vehicle_yaw)
            waypoints.append((x, y))
        
        self.get_logger().info(f"경로 생성: 차량 위치 ({self.vehicle_x:.2f}, {self.vehicle_y:.2f}), yaw: {self.vehicle_yaw:.2f}")
        self.get_logger().info(f"시작점 ({start_x:.2f}, {start_y:.2f}) → 끝점 ({waypoints[-1][0]:.2f}, {waypoints[-1][1]:.2f})")
        
        # waypoints를 PoseStamped 메시지로 변환
        for i, (x, y) in enumerate(waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header = Header()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            
            # Map origin을 더한 절대 좌표로 변환
            pose_stamped.pose.position.x = x + self.map_origin_x
            pose_stamped.pose.position.y = y + self.map_origin_y
            pose_stamped.pose.position.z = 0.0
            
            # 방향은 차량의 진행방향과 동일하게 설정
            yaw = self.vehicle_yaw
            
            # Quaternion 생성 (yaw만 사용)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose_stamped)
        
        # Path 퍼블리시
        self.path_pub.publish(path_msg)
        self.path_published = True
        
        self.get_logger().info(f"더미 local path 퍼블리시 완료: {len(path_msg.poses)}개 waypoints (0.5m 간격)")
        self.get_logger().info("경로: 차량 진행방향 기준 오른쪽 1m 지점에서 전방 20m까지")
        
        # 타이머 중지
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DummyLocalPathPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
