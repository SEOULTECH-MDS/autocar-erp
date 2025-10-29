#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Int32, Bool
from nav_msgs.msg import Odometry
from planning_msgs.msg import ModeState


class DeliveryPlanner(Node):
    """
    배달 미션 플래너 (표지판 좌표 입력 제거 버전)

    입력 토픽:
    - /target_sign (Int32): 표지판 ID (1-6)
    - /mode_state (ModeState): 현재 모드 상태
    - /autocar/location (Odometry): 차량 속도

    출력 토픽:
    - /pickup_complete_flag (Bool): 상차 완료 플래그
    - /delivery_complete_flag (Bool): 하차 완료 플래그

    완료 조건:
    - (옵션) 차량 속도 <= velocity_threshold
    - 일정 시간(4초) 유지 시 상차/하차 완료 플래그 발행
    """

    def __init__(self):
        super().__init__('delivery_planner')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('use_velocity_check', True)   # 속도 체크 사용 여부
        self.declare_parameter('velocity_threshold', 0.05)   # 정지 판단 속도 임계값 (m/s)

        # ---------------- QoS ----------------
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ---------------- Publishers ----------------
        self.pickup_complete_pub = self.create_publisher(Bool, '/pickup_complete_flag', qos_transient_local)
        self.delivery_complete_pub = self.create_publisher(Bool, '/delivery_complete_flag', qos_transient_local)

        # ---------------- Subscribers ----------------
        self.target_sign_sub = self.create_subscription(Int32, '/target_sign', self._on_target_sign, 10)
        self.mode_state_sub = self.create_subscription(ModeState, '/mode_state', self._on_mode_state, 10)
        self.vehicle_pose_sub = self.create_subscription(Odometry, '/autocar/location', self._on_vehicle_pose, 10)

        # ---------------- 상태 ----------------
        self._is_active: bool = False
        self._current_mode: Optional[int] = None
        self._target_sign_id: Optional[int] = None

        self._current_velocity: float = 0.0  # 평면 속도 크기

        self._completion_timer = None
        self._timer_active: bool = False

        self._pickup_completed: bool = False
        self._dropoff_completed: bool = False

        self.get_logger().info('배달 플래너 초기화 완료')

    # ============================================================================
    # 콜백 함수들 (이벤트 수신 순서)
    # ============================================================================

    def _on_mode_state(self, msg: ModeState) -> None:
        """모드 상태에 따른 활성화 제어"""
        self._current_mode = msg.current_mode

        # 배달 모드가 아니면 비활성화
        if msg.current_mode != ModeState.DELIVERY:
            self._is_active = False
            self._cancel_completion_timer()
            return

        # 배달 모드일 때만 동작
        if not self._target_sign_id:
            self._is_active = False
            return

        # 상차/하차 완료 여부에 따라 활성화
        if 1 <= self._target_sign_id <= 3:
            self._is_active = not self._pickup_completed
        elif 4 <= self._target_sign_id <= 6:
            self._is_active = not self._dropoff_completed
        else:
            self._is_active = False

    def _on_target_sign(self, msg: Int32) -> None:
        """타깃 표지판 ID 갱신. 임무 전환 시 상태 리셋."""
        sign_id = int(msg.data)
        if 1 <= sign_id <= 6:
            if self._target_sign_id != sign_id:
                self._cancel_completion_timer()
                self._is_active = False
                self._pickup_completed = False
                self._dropoff_completed = False
            self._target_sign_id = sign_id
            self.get_logger().info(f'표지판 ID 수신: {sign_id} ({"상차" if sign_id <= 3 else "하차"})')

    def _on_vehicle_pose(self, msg: Odometry) -> None:
        """차량 속도 갱신 및 조건 평가(타이머 제어)"""
        if not self._is_active:
            return

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self._current_velocity = math.hypot(vx, vy)

        # 속도 조건 평가 및 타이머 제어
        if self._check_completion_conditions():
            self._start_completion_timer()
        else:
            self._cancel_completion_timer()

    # ============================================================================
    # 헬퍼 함수들 (상태 관리 및 타이머 제어)
    # ============================================================================

    def _check_completion_conditions(self) -> bool:
        """완료 조건: (옵션) 속도 조건"""
        if self.get_parameter('use_velocity_check').value:
            v_th = float(self.get_parameter('velocity_threshold').value)
            return self._current_velocity <= v_th
        return True

    def _start_completion_timer(self) -> None:
        """조건 만족 시 타이머 시작 (4초 유지 시 완료 처리)"""
        if not self._timer_active and self._check_completion_conditions():
            self._timer_active = True
            self._completion_timer = self.create_timer(4.0, self._completion_timer_cb, oneshot=True)
            self.get_logger().info('완료 타이머 시작 (4초)')

    def _cancel_completion_timer(self) -> None:
        """타이머 취소"""
        if self._timer_active and self._completion_timer:
            self._completion_timer.cancel()
            self._completion_timer = None
            self._timer_active = False
            self.get_logger().info('완료 타이머 취소')

    def _completion_timer_cb(self):
        """타이머 만료 → 완료 플래그 발행 (4초 조건 이미 충족)"""
        if self._completion_timer:
            self._completion_timer.cancel()
            self._completion_timer = None
        self._timer_active = False

        self._handle_completion()

    def _handle_completion(self) -> None:
        """상차/하차 완료 플래그 발행"""
        complete_msg = Bool(data=True)

        if self._target_sign_id and 1 <= self._target_sign_id <= 3:
            if not self._pickup_completed:
                self.pickup_complete_pub.publish(complete_msg)
                self.get_logger().info('상차 완료!')
                self._pickup_completed = True
        elif self._target_sign_id and 4 <= self._target_sign_id <= 6:
            if not self._dropoff_completed:
                self.delivery_complete_pub.publish(complete_msg)
                self.get_logger().info('하차 완료!')
                self._dropoff_completed = True

        # 플래너 비활성화
        self._is_active = False


def main(args=None):
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
