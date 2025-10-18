#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from std_msgs.msg import Bool

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseArray, PoseStamped, Transform, Vector3, Quaternion
from nav_msgs.msg import Odometry
from planning_msgs.msg import ModeState
from autocar_msgs.msg import Waypoint, WaypointArray  # waypoint 메시지 import

class DeliveryPlanner(Node):
    """
    배달 미션을 위한 플래너 클래스
    
    입력 토픽:
    - /deliverysign_spot (PoseArray): 표지판 3D 위치
    - /target_sign (Int32): 표지판 ID (1-6)
    - /mode_state (ModeState): 현재 모드 상태
    - /autocar/location (Odometry): 차량의 현재 위치 및 속도
    
    출력 토픽:
    - /delivery_target (PoseStamped): 계산된 배달 목적지
    - /waypoint (WaypointArray): 생성된 경로점 목록
    
    동작 설명:
    1. 표지판 인식 및 위치 처리
       - 표지판 3D 위치를 velodyne에서 map 좌표계로 변환
       - 표지판 ID에 따라 상차(1-3)/하차(4-6) 모드 결정
    
    2. 배달 목적지 계산
       - 표지판 기준 왼쪽 0.3m, 후방 1m 위치 계산
       - 목적지 방향은 표지판을 바라보도록 설정
    
    3. Waypoint 생성
       - 차량의 현재 위치와 배달 목적지를 기반으로 waypoint 생성
       - 상차/하차 상태에 따라 waypoint 모드 설정 (PICKUP/DROPOFF)
    
    4. 자동 업데이트
       - 새로운 차량 위치 수신 시 waypoint 업데이트
       - 새로운 배달 목적지 계산 시 waypoint 업데이트
    """
    
    def __init__(self):
        super().__init__('delivery_planner')
        
        # QoS 프로파일 설정
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publishers
        self.delivery_target_pub = self.create_publisher(
            PoseStamped, '/delivery_target', qos_transient_local)
        self.waypoint_pub = self.create_publisher(
            WaypointArray, '/waypoint', qos_transient_local)
        self.pickup_complete_pub = self.create_publisher(
            Bool, '/pickup_complete_flag', qos_transient_local)
        self.dropoff_complete_pub = self.create_publisher(
            Bool, '/delivery_complete_flag', qos_transient_local)  # selector.py와 동일한 토픽명 사용
            
        # Subscribers
        self.sign_spot_sub = self.create_subscription(
            PoseArray, '/deliverysign_spot', self._on_sign_spot, 10)
        self.target_sign_sub = self.create_subscription(
            Int32, '/target_sign', self._on_target_sign, 10)
        self.mode_state_sub = self.create_subscription(
            ModeState, '/mode_state', self._on_mode_state, 10)
        self.vehicle_pose_sub = self.create_subscription(
            Odometry, '/autocar/location', self._on_vehicle_pose, 10)
            
        # TF 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 플래너 상태
        self._is_active = False
        self._current_mode = None
        self._target_sign_id = None
        self._sign_position = None  # velodyne 프레임 기준 위치
        self._vehicle_pose = None  # 차량의 현재 위치
        self._delivery_target = None  # 계산된 배달 목적지
        
        # 로거 설정
        self.get_logger().info('배달 플래너가 초기화되었습니다.')
    
    def _on_sign_spot(self, msg: PoseArray) -> None:
        """
        표지판 위치 메시지 콜백 함수
        """
        if not msg.poses:
            return
            
        # 첫 번째 표지판 위치만 사용 (현재는 단일 표지판만 처리)
        self._sign_position = msg.poses[0]
        self._update_delivery_target()
        
    def _on_target_sign(self, msg: Int32) -> None:
        """
        표지판 ID 메시지 콜백 함수
        """
        sign_id = msg.data
        if 1 <= sign_id <= 6:
            self._target_sign_id = sign_id
            self.get_logger().info(f'표지판 ID 수신: {sign_id} ({"상차" if sign_id <= 3 else "하차"})')
            self._update_delivery_target()
        
    def _on_mode_state(self, msg: ModeState) -> None:
        """
        모드 상태 메시지 콜백 함수
        """
        self._current_mode = msg.current_mode
        self._is_active = msg.current_mode == ModeState.DELIVERY
        
    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        주어진 pose를 target_frame으로 변환
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time())
                
            # 변환된 pose 생성
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = target_frame
            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            
            # 위치 변환
            transformed_pose.pose.position.x = (
                pose.pose.position.x * transform.transform.rotation.w * transform.transform.rotation.w +
                transform.transform.translation.x
            )
            transformed_pose.pose.position.y = (
                pose.pose.position.y * transform.transform.rotation.w * transform.transform.rotation.w +
                transform.transform.translation.y
            )
            transformed_pose.pose.position.z = (
                pose.pose.position.z * transform.transform.rotation.w * transform.transform.rotation.w +
                transform.transform.translation.z
            )
            
            # 방향 변환 (쿼터니언 곱)
            transformed_pose.pose.orientation.x = (
                transform.transform.rotation.w * pose.pose.orientation.x +
                transform.transform.rotation.x * pose.pose.orientation.w +
                transform.transform.rotation.y * pose.pose.orientation.z -
                transform.transform.rotation.z * pose.pose.orientation.y
            )
            transformed_pose.pose.orientation.y = (
                transform.transform.rotation.w * pose.pose.orientation.y -
                transform.transform.rotation.x * pose.pose.orientation.z +
                transform.transform.rotation.y * pose.pose.orientation.w +
                transform.transform.rotation.z * pose.pose.orientation.x
            )
            transformed_pose.pose.orientation.z = (
                transform.transform.rotation.w * pose.pose.orientation.z +
                transform.transform.rotation.x * pose.pose.orientation.y -
                transform.transform.rotation.y * pose.pose.orientation.x +
                transform.transform.rotation.z * pose.pose.orientation.w
            )
            transformed_pose.pose.orientation.w = (
                transform.transform.rotation.w * pose.pose.orientation.w -
                transform.transform.rotation.x * pose.pose.orientation.x -
                transform.transform.rotation.y * pose.pose.orientation.y -
                transform.transform.rotation.z * pose.pose.orientation.z
            )
            
            return transformed_pose
            
        except TransformException as e:
            self.get_logger().warning(f'좌표 변환 실패: {str(e)}')
            return None
            
    def _calculate_delivery_target(self, sign_pose: PoseStamped) -> PoseStamped:
        """
        표지판 위치를 기준으로 배달 목적지 계산
        - 표지판 기준 왼쪽 0.3m, 후방 1m 위치
        """
        target = PoseStamped()
        target.header = sign_pose.header
        
        # 표지판의 방향을 나타내는 쿼터니언을 오일러 각도로 변환
        roll, pitch, yaw = self._quaternion_to_euler(
            sign_pose.pose.orientation.x,
            sign_pose.pose.orientation.y,
            sign_pose.pose.orientation.z,
            sign_pose.pose.orientation.w
        )
        
        # 목적지 위치 계산 (표지판 기준 왼쪽 0.3m, 후방 1m)
        target.pose.position.x = (
            sign_pose.pose.position.x +
            -1.0 * math.cos(yaw) +  # 후방 1m
            0.3 * math.cos(yaw + math.pi/2)  # 왼쪽 0.3m
        )
        target.pose.position.y = (
            sign_pose.pose.position.y +
            -1.0 * math.sin(yaw) +  # 후방 1m
            0.3 * math.sin(yaw + math.pi/2)  # 왼쪽 0.3m
        )
        target.pose.position.z = sign_pose.pose.position.z
        
        # 목적지의 방향은 표지판을 바라보도록 설정 (yaw + 180도)
        target.pose.orientation = self._euler_to_quaternion(roll, pitch, yaw + math.pi)
        
        return target
        
    def _on_vehicle_pose(self, msg: Odometry) -> None:
        """
        차량 위치 메시지 콜백 함수
        """
        self._vehicle_pose = msg.pose.pose
        
        # 목적지에 도착했는지 확인
        if self._check_arrival():
            self._handle_arrival()
        else:
            self._update_waypoints()
            
    def _check_arrival(self) -> bool:
        """
        차량이 목적지에 도착했는지 확인
        - 위치 오차: 0.1m 이내
        - 각도 오차: 10도 이내
        """
        if not self._is_active or not self._delivery_target or not self._vehicle_pose:
            return False
            
        # 위치 오차 계산
        dx = self._vehicle_pose.position.x - self._delivery_target.pose.position.x
        dy = self._vehicle_pose.position.y - self._delivery_target.pose.position.y
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        # 각도 오차 계산
        target_roll, target_pitch, target_yaw = self._quaternion_to_euler(
            self._delivery_target.pose.orientation.x,
            self._delivery_target.pose.orientation.y,
            self._delivery_target.pose.orientation.z,
            self._delivery_target.pose.orientation.w
        )
        
        current_roll, current_pitch, current_yaw = self._quaternion_to_euler(
            self._vehicle_pose.orientation.x,
            self._vehicle_pose.orientation.y,
            self._vehicle_pose.orientation.z,
            self._vehicle_pose.orientation.w
        )
        
        # 각도 차이를 -180~180도 범위로 정규화
        angle_error = math.degrees(abs(current_yaw - target_yaw))
        if angle_error > 180:
            angle_error = 360 - angle_error
            
        # 위치와 각도 모두 허용 범위 내인지 확인
        return distance_error <= 0.1 and angle_error <= 10.0
        
    def _handle_arrival(self) -> None:
        """
        목적지 도착 시 처리
        - 상차/하차 완료 플래그 발행
        - waypoint 발행 중단
        """
        # 빈 waypoint 발행하여 이동 중단
        empty_waypoints = WaypointArray()
        empty_waypoints.header.frame_id = 'map'
        empty_waypoints.header.stamp = self.get_clock().now().to_msg()
        self.waypoint_pub.publish(empty_waypoints)
        
        # 상차/하차 완료 플래그 발행
        complete_msg = Bool()
        complete_msg.data = True
        
        if self._target_sign_id and 1 <= self._target_sign_id <= 3:
            self.pickup_complete_pub.publish(complete_msg)
            self.get_logger().info('상차 완료!')
        elif self._target_sign_id and 4 <= self._target_sign_id <= 6:
            self.dropoff_complete_pub.publish(complete_msg)
            self.get_logger().info('하차 완료!')
        
    def _update_delivery_target(self) -> None:
        """
        배달 목적지를 계산하고 발행
        """
        if not self._is_active or not self._sign_position:
            return
            
        # 표지판 위치를 PoseStamped로 변환
        sign_pose = PoseStamped()
        sign_pose.header.frame_id = 'velodyne'
        sign_pose.header.stamp = self.get_clock().now().to_msg()
        sign_pose.pose = self._sign_position
        
        # velodyne → map 좌표계 변환
        map_sign_pose = self._transform_pose(sign_pose, 'map')
        if not map_sign_pose:
            return
            
        # 배달 목적지 계산 및 발행
        self._delivery_target = self._calculate_delivery_target(map_sign_pose)
        self.delivery_target_pub.publish(self._delivery_target)
        self.get_logger().info('배달 목적지가 업데이트되었습니다.')
        
        # waypoint 업데이트
        self._update_waypoints()
        
    def _update_waypoints(self) -> None:
        """
        차량의 현재 위치와 배달 목적지를 이용하여 waypoint 생성 및 발행
        """
        if not self._is_active or not self._delivery_target or not self._vehicle_pose:
            return
            
        # WaypointArray 메시지 생성
        waypoints = WaypointArray()
        waypoints.header.frame_id = 'map'
        waypoints.header.stamp = self.get_clock().now().to_msg()
        
        # 현재 위치에서 목적지까지의 waypoint 생성
        waypoint = Waypoint()
        waypoint.pose = self._delivery_target.pose
        
        # 상차/하차 상태에 따라 waypoint 설정
        if self._target_sign_id and 1 <= self._target_sign_id <= 3:
            waypoint.mode = Waypoint.PICKUP  # 상차 모드
        elif self._target_sign_id and 4 <= self._target_sign_id <= 6:
            waypoint.mode = Waypoint.DROPOFF  # 하차 모드
        else:
            waypoint.mode = Waypoint.NORMAL  # 일반 주행
            
        waypoints.waypoints.append(waypoint)
        
        # Waypoint 발행
        self.waypoint_pub.publish(waypoints)
        self.get_logger().info('Waypoint가 업데이트되었습니다.')
        
    def _quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
        """쿼터니언을 오일러 각도로 변환"""
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        
        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
        
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """오일러 각도를 쿼터니언으로 변환"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q


def main(args=None):
    """배달 플래너 노드의 메인 함수"""
    rclpy.init(args=args)
    
    try:
        node = DeliveryPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()