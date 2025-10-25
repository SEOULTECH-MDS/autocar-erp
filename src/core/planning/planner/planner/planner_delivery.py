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

from std_msgs.msg import Int32, Bool, Float64
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from planning_msgs.msg import ModeState

class DeliveryPlanner(Node):
    """
    배달 미션을 위한 플래너 클래스
    
    입력 토픽:
    - /deliverysign_spot (PoseArray): 표지판 3D 위치 (velodyne 좌표계)
    - /target_sign (Int32): 표지판 ID (1-6)
    - /mode_state (ModeState): 현재 모드 상태
    
    출력 토픽:
    - /delivery_distance (Float64): 표지판까지의 거리 (velodyne 좌표계 기준)
    - /pickup_fin (Bool): 상차 완료 플래그
    - /dropoff_fin (Bool): 하차 완료 플래그
    
    동작 설명:
    1. 표지판 인식 및 거리 계산
       - 표지판 3D 위치로부터 거리 계산 (velodyne 좌표계 기준)
       - 표지판 ID에 따라 상차(1-3)/하차(4-6) 모드 결정
    
    2. 완료 조건 확인
       - 거리가 2.0m 이하이거나
       - 차량 속도가 임계값(0.01 m/s) 이하일 때 (옵션)
    
    3. 완료 처리
       - 조건 만족 시 4초 타이머 시작
       - 타이머 완료 후 상차/하차 완료 플래그 발행
    """
    
    def __init__(self):
        super().__init__('delivery_planner')

        # 파라미터 선언
        self.declare_parameter('use_velocity_check', True)  # 속도 체크 사용 여부
        self.declare_parameter('velocity_threshold', 0.01)   # 정지 판단 속도 임계값 (m/s)
        
        # QoS 프로파일 설정
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publishers
        self.distance_pub = self.create_publisher(
            Float64, '/delivery_distance', qos_transient_local)
        self.pickup_complete_pub = self.create_publisher(
            Bool, '/pickup_complete_flag', qos_transient_local)
        self.delivery_complete_pub = self.create_publisher(
            Bool, '/delivery_complete_flag', qos_transient_local)
            
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
        self._completion_timer = None  # 완료 처리 타이머
        self._current_velocity = 0.0  # 현재 차량 속도
        self._pickup_completed = False    # 상차 완료 플래그 발행 여부
        self._dropoff_completed = False   # 하차 완료 플래그 발행 여부
        
        # 로거 설정
        self.get_logger().info('배달 플래너가 초기화되었습니다.')
    
    def _calculate_distance(self, pose: Pose) -> float:
        """
        velodyne 좌표계 기준 원점으로부터의 거리 계산
        """
        return math.sqrt(
            pose.position.x**2 + 
            pose.position.y**2 + 
            pose.position.z**2
        )

    def _on_vehicle_pose(self, msg: Odometry) -> None:
        """
        차량 위치 메시지 콜백 함수
        속도가 임계값 이하이고 파라미터가 활성화되어 있으면 완료 처리 시작
        """
        if not self._is_active:
            return
            
        # 속도 업데이트
        self._current_velocity = abs(msg.twist.twist.linear.x)
        
        # 속도 체크가 활성화되어 있고 속도가 임계값 이하인 경우
        use_velocity_check = self.get_parameter('use_velocity_check').value
        if use_velocity_check:
            velocity_threshold = self.get_parameter('velocity_threshold').value
            if self._current_velocity <= velocity_threshold:
                self._start_completion_timer()

    def _start_completion_timer(self) -> None:
        """
        4초 후에 완료 처리를 수행하는 타이머 시작 (one-shot)
        """
        if self._completion_timer is None:
            self._completion_timer = self.create_timer(4.0, self._handle_completion)
            self.get_logger().info('완료 타이머 시작 (4초)')

    def _handle_completion(self) -> None:
        """
        완료 플래그 발행 처리 (상차/하차 각각 한 번만 발행)
        """
        # 타이머를 one-shot으로 만들기 위해 콜백 시작에서 취소
        if self._completion_timer is not None:
            self._completion_timer.cancel()
            self._completion_timer = None
        
        # 완료 메시지 생성
        complete_msg = Bool()
        complete_msg.data = True
        
        # 상차/하차 상태에 따라 적절한 플래그 발행
        if self._target_sign_id and 1 <= self._target_sign_id <= 3:
            if not self._pickup_completed:  # 상차 완료가 아직 안된 경우만
                self.pickup_complete_pub.publish(complete_msg)
                self.get_logger().info('상차 완료!')
                self._pickup_completed = True
        elif self._target_sign_id and 4 <= self._target_sign_id <= 6:
            if not self._dropoff_completed:  # 하차 완료가 아직 안된 경우만
                self.delivery_complete_pub.publish(complete_msg)
                self.get_logger().info('하차 완료!')
                self._dropoff_completed = True
        
        # 플래너 비활성화 (현재 동작 완료)
        self._is_active = False

    def _on_sign_spot(self, msg: PoseArray) -> None:
        """
        표지판 위치 메시지 콜백 함수
        """
        if not msg.poses or not self._is_active:
            return
            
        # 첫 번째 표지판 위치만 사용 (현재는 단일 표지판만 처리)
        self._sign_position = msg.poses[0]
        
        # 거리 계산 및 발행
        distance = self._calculate_distance(self._sign_position)
        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)
        
        # 거리가 2.0m 이하이면 완료 타이머 시작
        if distance <= 2.0:
            self._start_completion_timer()
        
    def _on_target_sign(self, msg: Int32) -> None:
        """
        표지판 ID 메시지 콜백 함수
        """
        sign_id = msg.data
        if 1 <= sign_id <= 6:
            self._target_sign_id = sign_id
            self.get_logger().info(f'표지판 ID 수신: {sign_id} ({"상차" if sign_id <= 3 else "하차"})')
            self._update_delivery_target()
    
    def _update_delivery_target(self) -> None:
        # 새로운 표지판 수신 시 진행 중 타이머는 취소
        if self._completion_timer is not None:
            self._completion_timer.cancel()
            self._completion_timer = None

        # 현재 모드가 배달 모드일 때만 활성화 상태 갱신
        if self._current_mode == ModeState.DELIVERY:
            if self._target_sign_id and 1 <= self._target_sign_id <= 3:
                # 상차 표지판: 상차 미완료면 활성화
                self._is_active = not self._pickup_completed
            elif self._target_sign_id and 4 <= self._target_sign_id <= 6:
                # 하차 표지판: 하차 미완료면 활성화
                self._is_active = not self._dropoff_completed
            else:
                # 유효한 표지판을 아직 못받았으면 대기/활성화 정책
                self._is_active = True
        else:
            self._is_active = False
        
    def _on_mode_state(self, msg: ModeState) -> None:
        """
        모드 상태 메시지 콜백 함수
        """
        self._current_mode = msg.current_mode
        
        # 배달 모드가 켜질 때만 활성화 (해당 동작이 아직 완료되지 않은 경우)
        if msg.current_mode == ModeState.DELIVERY:
            if self._target_sign_id and 1 <= self._target_sign_id <= 3:
                # 상차 표지판이고 상차가 아직 안된 경우
                self._is_active = not self._pickup_completed
            elif self._target_sign_id and 4 <= self._target_sign_id <= 6:
                # 하차 표지판이고 하차가 아직 안된 경우
                self._is_active = not self._dropoff_completed
            else:
                # 표지판이 아직 인식되지 않은 경우
                self._is_active = True
        else:
            self._is_active = False
        
            
        
        


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