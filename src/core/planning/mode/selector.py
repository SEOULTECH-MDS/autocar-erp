#!/usr/bin/env python3

"""모드 셀렉터 노드
- 차량 위치/미션/신호/정지선 정보를 종합해 `ModeState`를 퍼블리시합니다.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Int32, Int32MultiArray, Float64
from planning_msgs.msg import ModeState


TRAFFIC_APPROACH = 'APPROACH'
TRAFFIC_DECELERATE = 'DECELERATE'
TRAFFIC_STOPPED = 'STOPPED'
TRAFFIC_GO = 'GO'


class ModeSelector(Node):
    """모드 셀렉터

    입력: `/traffic_sign`, `/stopline_distance`, `/stopline_type`, `/mission_zone`, `/open_parking_area_id`
    출력: `/mode_state`, `/traffic_flag`
    """

    def __init__(self) -> None:
        super().__init__('mode_selector')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('yellow_treat_as_red', False)
        self.declare_parameter('stopline_margin', 1.5)  # [m]
        self.declare_parameter('traffic_confirm_window', 0.5)  # [s]
        self.declare_parameter('default_mode', int(ModeState.DRIVE))
        # (path follower 힌트 제거됨)

        # Publishers
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(ModeState, '/mode_state', 10)
        self.traffic_flag_pub = self.create_publisher(String, '/traffic_flag', qos_transient_local)
        # (path follower 힌트 퍼블리셔 제거됨)

        # Subscriptions (traffic)
        self.tl_sub = self.create_subscription(String, '/traffic_sign', self._traffic_cb, 10)
        self.stopline_dist_sub = self.create_subscription(Float64, '/stopline_distance', self._stopline_dist_cb, 10)
        self.stopline_type_sub = self.create_subscription(String, '/stopline_type', self._stopline_type_cb, 10)

        # Subscriptions (mission/zone)
        self.mission_zone_sub = self.create_subscription(String, '/mission_zone', self._mission_zone_cb, 10)
        # Optional: parking open area id can hint we are in parking mission context
        self.open_area_sub = self.create_subscription(Int32, '/open_parking_area_id', self._open_area_cb, 10)
        # Optional: delivery cues (if needed)
        self.delivery_sign_sub = self.create_subscription(Int32MultiArray, '/delivery_sign', self._delivery_sign_cb, 10)

        # Internal state
        self.mode_descriptions = {
            int(ModeState.DRIVE): 'DRIVE',
            int(ModeState.PAUSE): 'PAUSE',
            int(ModeState.OBSTACLE_STATIC): 'OBSTACLE_STATIC',
            int(ModeState.OBSTACLE_DYNAMIC): 'OBSTACLE_DYNAMIC',
            int(ModeState.DELIVERY): 'DELIVERY',
            int(ModeState.PARKING): 'PARKING',
            int(ModeState.RETURN): 'RETURN',
        }

        self.current_mode: int = int(self.get_parameter('default_mode').value)
        self.mission_zone: str = 'road'  # road|parking|delivery
        self._parking_hint_active: bool = False

        # Traffic inputs
        self.tl_state_raw: Optional[str] = None
        self.tl_state_confirmed: Optional[str] = None
        self.tl_current_raw_since = None
        self.tl_last_seen = None
        self.stopline_distance: float = float('inf')
        self.stopline_type: str = 'no_stopline'

        # Derived flags
        self.traffic_flag: str = TRAFFIC_GO

        # Timer
        period = 1.0 / max(1e-3, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._on_timer)

        self.get_logger().info('Mode Selector node started.')

    # ─────────────── Subscriptions ───────────────
    def _traffic_cb(self, msg: String) -> None:
        raw = (msg.data or '').strip().lower()
        self.tl_state_raw = raw
        now = self.get_clock().now()
        if (self.tl_state_confirmed or 'none') != raw:
            self.tl_current_raw_since = now
        # Confirm after window
        win = float(self.get_parameter('traffic_confirm_window').value)
        if self.tl_current_raw_since is not None:
            elapsed = (now - self.tl_current_raw_since).nanoseconds * 1e-9
            if elapsed >= win:
                self.tl_state_confirmed = raw
                self.tl_last_seen = now

    def _stopline_dist_cb(self, msg: Float64) -> None:
        try:
            self.stopline_distance = float(msg.data)
        except Exception:
            self.stopline_distance = float('inf')

    def _stopline_type_cb(self, msg: String) -> None:
        self.stopline_type = (msg.data or 'no_stopline').strip()

    def _mission_zone_cb(self, msg: String) -> None:
        zone = (msg.data or '').strip().lower()
        if zone in ('road', 'parking', 'delivery'):
            self.mission_zone = zone

    def _open_area_cb(self, msg: Int32) -> None:
        # If any valid open area id is published, treat as parking context hint
        try:
            self._parking_hint_active = int(msg.data) >= 0
        except Exception:
            self._parking_hint_active = False

    def _delivery_sign_cb(self, msg: Int32MultiArray) -> None:
        # Optional hook: if delivery_start detected, switch zone to delivery
        # Expecting custom encoding from perception; keep this as a weak hint
        if self.mission_zone != 'delivery':
            # No-op unless user wants automatic switching via this topic
            pass

    # ─────────────── Timer loop ───────────────
    def _on_timer(self) -> None:
        """주기 작업: 트래픽 플래그 갱신 → 모드 결정 → 스테이지 힌트 퍼블리시"""
        self._update_traffic_flag()
        self._select_and_publish_mode()
        # (path follower 힌트 퍼블리시 제거됨)

    # ─────────────── Traffic flag derivation ───────────────
    def _is_stopline_relevant(self) -> bool:
        """정지선 타입이 신호와 관련 있는지 여부"""
        t = (self.stopline_type or '').strip().lower()
        return t in ('traffic_straight', 'traffic_left', 'traffic_right')

    def _is_go_signal(self) -> bool:
        """신호가 '진행(녹색)'인지 확인"""
        tokens = (self.tl_state_confirmed or '').split()
        return any(tok in ('green', 'go') for tok in tokens)

    def _is_red_like_signal(self) -> bool:
        """신호가 '정지(빨강/선택적 노랑)'인지 확인"""
        tokens = (self.tl_state_confirmed or '').split()
        yellow_as_red = bool(self.get_parameter('yellow_treat_as_red').value)
        return ('red' in tokens) or (yellow_as_red and 'yellow' in tokens)

    def _update_traffic_flag(self) -> None:
        """정지선 거리/타입과 신호로 트래픽 플래그(APPROACH/DECELERATE/STOPPED/GO) 계산"""
        if not self._is_stopline_relevant():
            self.traffic_flag = TRAFFIC_GO
        else:
            d = self.stopline_distance
            margin = float(self.get_parameter('stopline_margin').value)
            if self._is_red_like_signal():
                if d <= margin:
                    self.traffic_flag = TRAFFIC_STOPPED
                elif d < max(3.0, 2.0 * margin):
                    self.traffic_flag = TRAFFIC_DECELERATE
                else:
                    self.traffic_flag = TRAFFIC_APPROACH
            elif self._is_go_signal():
                self.traffic_flag = TRAFFIC_GO
            else:
                # Unknown signal: behave cautiously when close
                if d <= margin:
                    self.traffic_flag = TRAFFIC_STOPPED
                elif d < max(3.0, 2.0 * margin):
                    self.traffic_flag = TRAFFIC_DECELERATE
                else:
                    self.traffic_flag = TRAFFIC_APPROACH

        self.traffic_flag_pub.publish(String(data=self.traffic_flag))

    # ─────────────── Mode selection ───────────────
    def _select_and_publish_mode(self) -> None:
        """미션 구역/주차 힌트/트래픽 플래그로 `ModeState` 결정 후 퍼블리시"""
        # Determine mission context
        mission = self.mission_zone
        if mission == 'road' and self._parking_hint_active:
            mission = 'parking'

        # Base mode decision
        selected_mode = int(ModeState.DRIVE)
        if mission == 'parking':
            selected_mode = int(ModeState.PARKING)
        elif mission == 'delivery':
            selected_mode = int(ModeState.DELIVERY)
        else:
            # Road: consider traffic for PAUSE override near stoplines
            if self.traffic_flag in (TRAFFIC_DECELERATE, TRAFFIC_STOPPED):
                selected_mode = int(ModeState.PAUSE)
            else:
                selected_mode = int(ModeState.DRIVE)

        # Publish if changed or periodically
        if selected_mode != self.current_mode:
            self.current_mode = selected_mode

        msg = ModeState()
        msg.current_mode = int(self.current_mode)
        msg.description = self.mode_descriptions.get(int(self.current_mode), 'UNKNOWN')
        self.mode_pub.publish(msg)

    # (path follower 스테이지 힌트 관련 함수 제거됨)


def main(args=None):
    """엔트리 포인트: 모드 셀렉터 노드 실행"""
    rclpy.init(args=args)
    node = ModeSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


