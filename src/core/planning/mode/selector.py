#!/usr/bin/env python3

"""모드 셀렉터 노드 (v2.0)
- 차량의 현재 상황을 종합하여 5가지 주행 모드를 결정합니다.
- DRIVING(기본), PAUSE(신호대기), PARKING(주차), 배달_상차, 배달_하차
"""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Int32, Int64, Bool
from planning_msgs.msg import ModeState


class ModeType:
    """모드 타입 정의"""
    DRIVING = "DRIVING"              # 일반 주행 모드
    PAUSE = "PAUSE"                  # 신호 대기 모드  
    PARKING = "PARKING"              # 주차 모드
    DELIVERY_PICKUP = "배달_상차"     # 배달 상차 모드
    DELIVERY_DROPOFF = "배달_하차"    # 배달 하차 모드


class ModeSelector(Node):
    """모드 셀렉터 v2.0
    
    입력 토픽:
    - /traffic_sign: 교통 신호 상태 (String: "Green", "Red", "Left", "Straightleft")
    - /current_lanelet_id: 현재 Lanelet ID (Int64) - Map ID로 변환됨
    - /target_sign: 표지판 인식 (Int32: 1,2,3=상차 | 4,5,6=하차)
    - /parking_complete_flag: 주차 미션 완료 (Bool)
    
    출력 토픽:
    - /mode_state: 현재 모드 상태 (ModeState)
    """

    def __init__(self) -> None:
        super().__init__('mode_selector_v2')

        # ===========================================
        # 파라미터 설정
        # ===========================================
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('default_mode', ModeType.DRIVING)
        
        # 구역 ID 설정 (리스트로 유연하게 관리)
        self.declare_parameter('parking_zone_ids', [1, 2, 3])
        self.declare_parameter('dropoff_zone_ids', [4, 5, 6])

        # ===========================================
        # Publisher 설정
        # ===========================================
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(ModeState, '/mode_state', qos_transient_local)

        # ===========================================
        # Subscriber 설정
        # ===========================================
        # 교통 신호
        self.traffic_sign_sub = self.create_subscription(
            String, '/traffic_sign', self._traffic_sign_cb, 10)
        
        # 위치/맵 정보 (Lanelet ID를 Map ID로 변환)
        self.lanelet_id_sub = self.create_subscription(
            Int64, '/current_lanelet_id', self._lanelet_id_cb, 10)
        
        # 표지판 인식 (통합)
        self.target_sign_sub = self.create_subscription(
            Int32, '/target_sign', self._target_sign_cb, 10)
        
        # 미션 완료 플래그
        self.parking_completed_sub = self.create_subscription(
            Bool, '/parking_complete_flag', self._parking_completed_cb, 10)
        self.delivery_completed_sub = self.create_subscription(
            Bool, '/delivery_complete_flag', self._delivery_completed_cb, 10)

        # ===========================================
        # 내부 상태 변수
        # ===========================================
        self.current_mode: str = str(self.get_parameter('default_mode').value)
        self.previous_mode: str = self.current_mode
        
        # 입력 상태들
        self.traffic_sign: Optional[str] = None             # "Green", "Red", "Left", "Straightleft"
        self.current_map_id: Optional[int] = None
        self.target_sign: Optional[int] = None              # 1,2,3=상차 | 4,5,6=하차
        self.parking_complete_flag: bool = False
        self.delivery_complete_flag: bool = False
        
        # 구역 ID 리스트 가져오기
        self.parking_zone_ids: List[int] = list(self.get_parameter('parking_zone_ids').value)
        self.dropoff_zone_ids: List[int] = list(self.get_parameter('dropoff_zone_ids').value)
        
        # 미션 상태 플래그 (한 번만 수행되는 미션들)
        self.parking_mission_started: bool = False
        self.delivery_mission_started: bool = False
        
        # 모드 매핑 (String → ModeState 상수)
        self.mode_mapping = {
            ModeType.DRIVING: ModeState.DRIVE,
            ModeType.PAUSE: ModeState.PAUSE,
            ModeType.PARKING: ModeState.PARKING,
            ModeType.DELIVERY_PICKUP: ModeState.DELIVERY,
            ModeType.DELIVERY_DROPOFF: ModeState.DELIVERY,
        }

        # ===========================================
        # 타이머 설정
        # ===========================================
        period = 1.0 / max(1e-3, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._on_timer)

        self.get_logger().info(f'Mode Selector v2.0 started.')
        self.get_logger().info(f'Parking zones: {self.parking_zone_ids}')
        self.get_logger().info(f'Dropoff zones: {self.dropoff_zone_ids}')

    # ===========================================
    # 콜백 함수들
    # ===========================================
    def _traffic_sign_cb(self, msg: String) -> None:
        """교통 신호 수신"""
        signal = (msg.data or '').strip()
        if signal in ('Green', 'Red', 'Left', 'Straightleft'):
            self.traffic_sign = signal
            self.get_logger().debug(f'Traffic sign: {signal}')

    def _lanelet_id_cb(self, msg: Int64) -> None:
        """현재 Lanelet ID 수신 및 Map ID로 변환"""
        lanelet_id = int(msg.data)
        self.current_map_id = self._lanelet_to_map_id(lanelet_id)
        self.get_logger().debug(f'Current lanelet ID: {lanelet_id}, Map ID: {self.current_map_id}')

    def _target_sign_cb(self, msg: Int32) -> None:
        """표지판 인식 수신 (1,2,3=상차 | 4,5,6=하차)"""
        sign_id = int(msg.data)
        if 1 <= sign_id <= 6:
            self.target_sign = sign_id
            if 1 <= sign_id <= 3:
                self.get_logger().info(f'상차 표지판 인식됨! (ID: {sign_id})')
            elif 4 <= sign_id <= 6:
                self.get_logger().info(f'하차 표지판 인식됨! (ID: {sign_id})')
        else:
            self.get_logger().warn(f'알 수 없는 표지판 ID: {sign_id}')

    def _parking_completed_cb(self, msg: Bool) -> None:
        """주차 미션 완료 플래그 수신"""
        if bool(msg.data):
            self.parking_complete_flag = True
            self.get_logger().info('주차 미션 완료!')

    def _delivery_completed_cb(self, msg: Bool) -> None:
        """배달 미션 완료 플래그 수신"""
        if bool(msg.data):
            self.delivery_complete_flag = True
            self.get_logger().info('배달 미션 완료!')

    # ===========================================
    # 메인 로직
    # ===========================================
    def _on_timer(self) -> None:
        """주기적으로 모드를 결정하고 퍼블리시"""
        new_mode = self._determine_mode()
        
        if new_mode != self.current_mode:
            self.previous_mode = self.current_mode
            self.current_mode = new_mode
            self.get_logger().info(f'모드 변경: {self.previous_mode} → {self.current_mode}')
        
        # 모드 상태 퍼블리시
        msg = ModeState()
        msg.current_mode = self.mode_mapping.get(self.current_mode, ModeState.DRIVE)
        msg.description = self.current_mode
        self.mode_pub.publish(msg)

    def _determine_mode(self) -> str:
        """현재 상황을 종합하여 모드 결정"""
        
        # ===========================================
        # 1. 미션 완료 처리 (최우선)
        # ===========================================
        if self.parking_complete_flag and self.current_mode == ModeType.PARKING:
            self.parking_complete_flag = False  # 플래그 리셋
            self.parking_mission_started = False
            return ModeType.DRIVING

        # ===========================================
        # 2. 신호등 처리 (PAUSE 모드)
        # ===========================================
        if self.traffic_sign == "Red" and self.current_mode == ModeType.DRIVING:
            return ModeType.PAUSE
            
        if self.traffic_sign in ("Green", "Left", "Straightleft") and self.current_mode == ModeType.PAUSE:
            return ModeType.DRIVING

        # ===========================================
        # 3. 주차 미션 처리
        # ===========================================
        if (self.current_map_id is not None and 
            self.current_map_id in self.parking_zone_ids and 
            not self.parking_mission_started and
            self.current_mode == ModeType.DRIVING):
            
            self.parking_mission_started = True
            return ModeType.PARKING

        # ===========================================
        # 4. 배달 미션 처리
        # ===========================================
        # 상차 모드 시작 조건 (target_sign 1,2,3)
        if (self.target_sign is not None and 
            1 <= self.target_sign <= 3 and
            not self.delivery_mission_started and
            self.current_mode == ModeType.DRIVING):
            
            self.delivery_mission_started = True
            self.target_sign = None  # 플래그 리셋
            return ModeType.DELIVERY_PICKUP

        # 하차 모드 전환 조건 (target_sign 4,5,6 + 하차구역)
        if (self.current_mode == ModeType.DELIVERY_PICKUP and
            self.current_map_id is not None and
            self.current_map_id in self.dropoff_zone_ids and
            self.target_sign is not None and
            4 <= self.target_sign <= 6):
            
            self.target_sign = None  # 플래그 리셋
            return ModeType.DELIVERY_DROPOFF
            
        # 배달 종료 조건 (하차모드에서 배달 종료 플래그 수신 시)
        if self.delivery_complete_flag and self.current_mode == ModeType.DELIVERY_DROPOFF:
            self.delivery_complete_flag = False  # 플래그 리셋
            self.delivery_mission_started = False
            return ModeType.DRIVING

        # ===========================================
        # 5. 현재 모드 유지
        # ===========================================
        return self.current_mode

    def _is_in_parking_zone(self) -> bool:
        """현재 주차 구역에 있는지 확인"""
        return (self.current_map_id is not None and 
                self.current_map_id in self.parking_zone_ids)

    def _is_in_dropoff_zone(self) -> bool:
        """현재 하차 구역에 있는지 확인"""
        return (self.current_map_id is not None and 
                self.current_map_id in self.dropoff_zone_ids)
    
    def _lanelet_to_map_id(self, lanelet_id: int) -> int:
        """Lanelet ID를 Map ID로 변환
        
        Args:
            lanelet_id: 현재 차량이 위치한 Lanelet ID
            
        Returns:
            int: 해당하는 Map ID (구역 번호)
            
        Note:
            mirae_map의 실제 Lanelet ID (1~8)를 기반으로 한 매핑입니다.
            맵 구성이 변경되면 이 매핑도 함께 수정해야 합니다.
        """
        # mirae_map 실제 Lanelet ID 범위: 1~8
        
        # 주차 구역 매핑 (parking_zone_ids: [1, 2, 3])
        if lanelet_id == 1:
            return 1  # 주차구역 1
        elif lanelet_id == 2:
            return 2  # 주차구역 2  
        elif lanelet_id == 3:
            return 3  # 주차구역 3
            
        # 하차 구역 매핑 (dropoff_zone_ids: [4, 5, 6])
        elif lanelet_id == 4:
            return 4  # 하차구역 1
        elif lanelet_id == 5:
            return 5  # 하차구역 2
        elif lanelet_id == 6:
            return 6  # 하차구역 3
            
        # 일반 도로 구역
        elif lanelet_id == 7:
            return 7  # 일반 도로 1
        elif lanelet_id == 8:
            return 8  # 일반 도로 2
            
        # 기타 구역 (예상치 못한 Lanelet ID)
        else:
            self.get_logger().warn(f'Unknown lanelet_id: {lanelet_id}, returning default map_id: 0')
            return 0  # 기본값

    def get_current_mode_info(self) -> dict:
        """현재 모드 상태 정보를 딕셔너리로 반환 (디버깅용)"""
        return {
            'current_mode': self.current_mode,
            'traffic_sign': self.traffic_sign,
            'map_id': self.current_map_id,
            'target_sign': self.target_sign,
            'parking_completed': self.parking_complete_flag,
            'in_parking_zone': self._is_in_parking_zone(),
            'in_dropoff_zone': self._is_in_dropoff_zone(),
        }


def main(args=None):
    """엔트리 포인트: 모드 셀렉터 v2.0 노드 실행"""
    rclpy.init(args=args)
    node = ModeSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('모드 셀렉터 종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()