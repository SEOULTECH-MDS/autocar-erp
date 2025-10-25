#!/usr/bin/env python3
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Int32, Int64, Bool, Float64
from nav_msgs.msg import Odometry
from planning_msgs.msg import ModeState


class CompetitionType:
    """대회 타입 정의"""
    QUALIFYING = "qualifying"       # 예선
    FINAL = "final"                 # 본선

class MapType:
    """맵 타입 정의"""
    KCITY = "kcity"                 # K-City 맵
    MIRAE = "mirae"                 # 미래관 맵

class QualifyingMode:
    """예선 모드 정의"""
    QUALIFYING_DRIVING = "QUALIFYING_DRIVING"        # 예선 일반 주행
    QUALIFYING_PARKING = "QUALIFYING_PARKING"        # 예선 주차
    QUALIFYING_UTURN = "QUALIFYING_UTURN"            # 예선 유턴
    QUALIFYING_GPS_OFF = "QUALIFYING_GPS_OFF"        # 예선 GPS 차단

class FinalMode:
    """본선 모드 정의"""
    FINAL_DRIVING = "FINAL_DRIVING"                  # 본선 일반 주행
    FINAL_PARKING = "FINAL_PARKING"                  # 본선 주차
    FINAL_PAUSE = "FINAL_PAUSE"                      # 본선 신호 대기
    FINAL_DELIVERY_PICKUP = "FINAL_배달_상차"         # 본선 배달 상차
    FINAL_DELIVERY_DROPOFF = "FINAL_배달_하차"        # 본선 배달 하차

class ModeType:
    """통합 모드 접근용 (하위 호환성)"""
    # 예선 모드들
    QUALIFYING_DRIVING = QualifyingMode.QUALIFYING_DRIVING
    QUALIFYING_PARKING = QualifyingMode.QUALIFYING_PARKING
    QUALIFYING_UTURN = QualifyingMode.QUALIFYING_UTURN
    QUALIFYING_GPS_OFF = QualifyingMode.QUALIFYING_GPS_OFF
    
    # 본선 모드들
    FINAL_DRIVING = FinalMode.FINAL_DRIVING
    FINAL_PARKING = FinalMode.FINAL_PARKING
    FINAL_PAUSE = FinalMode.FINAL_PAUSE
    FINAL_DELIVERY_PICKUP = FinalMode.FINAL_DELIVERY_PICKUP
    FINAL_DELIVERY_DROPOFF = FinalMode.FINAL_DELIVERY_DROPOFF


class ModeSelector(Node):
    """
    자율주행 차량의 모드를 관리하는 셀렉터 노드 v2.0
    
    ===========================================
    주요 기능:
    ===========================================
    1. 예선/본선 모드 분리: competition_type 파라미터로 구분
       - 예선: DRIVING, PARKING, UTURN, GPS_OFF
       - 본선: DRIVING, PAUSE, PARKING, DELIVERY(상차/하차)
    
    2. K-City/미래관 맵 지원: map_type 파라미터로 구분
       - K-City: 예선/본선 주차 구역 분리
       - 미래관: 통합 주차 구역
    
    3. 신호등 기반 정지 제어:
       - Red 신호: 즉시 PAUSE 모드
       - Green/Left/Straightleft: 0.3초 확인 후 DRIVING 모드
    
    4. 우회전 정지선 특수 처리 (본선):
       - 지정 구역(10)에서 virtual 정지선 감지 시 정지
       - 신호등 무시하고 차량 완전 정지(0.01 m/s) 후 3.2초 대기
       - 구역 이탈 시 타이머 자동 초기화
       - nonstop 정지선은 무시 (정지하지 않음)
    
    5. 배달 미션 (본선):
       - 상차(1-3) → 주행 → 하차(4-6) 시퀀스
       - 각 단계 완료 후 DRIVING 모드 복귀
    
    ===========================================
    입력 토픽:
    ===========================================
    - /traffic_sign: 교통 신호 상태 (String: "Green", "Red", "Left", "Straightleft")
    - /current_lanelet_id: 현재 Lanelet ID (Int64)
    - /target_sign: 표지판 인식 (Int32: 1,2,3=상차 | 4,5,6=하차)
    - /parking_complete_flag: 주차 미션 완료 (Bool)
    - /pickup_complete_flag: 상차 미션 완료 (Bool)
    - /delivery_complete_flag: 하차 미션 완료 (Bool)
    - /stopline_distance: 정지선까지의 거리 (Float64)
    - /stopline_type: 정지선 타입 (String: "right", "left", "straight", "nonstop", "no_stopline")
    - /autocar/location: 차량 위치 및 속도 정보 (Odometry)
    
    ===========================================
    출력 토픽:
    ===========================================
    - /mode_state: 현재 모드 상태 (ModeState)
      * DRIVE=0, PAUSE=1, DELIVERY=4, PARKING=5, UTURN=7, GPS_OFF=8
    
    ===========================================
    주요 파라미터:
    ===========================================
    - competition_type: "qualifying" | "final" (기본: final)
    - map_type: "kcity" | "mirae" (기본: kcity)
    - traffic_signal_confirm_duration: 신호등 확인 시간 (기본: 0.1초)
    - stopline_pause_duration: 우회전 정지 시간 (기본: 3.2초)
    - vehicle_stop_velocity_threshold: 정지 판단 속도 (기본: 0.01 m/s)
    """

    def __init__(self) -> None:
        super().__init__('mode_selector_v2')

        # ===========================================
        # 파라미터 설정
        # ===========================================
        self.declare_parameter('publish_rate', 10.0)
        
        # 대회 및 맵 구분 (먼저 선언)
        # self.declare_parameter('competition_type', CompetitionType.QUALIFYING)  # "qualifying" | "final" 예선 / 본선
        self.declare_parameter('competition_type', CompetitionType.FINAL)  # "qualifying" | "final" 예선 / 본선
        self.declare_parameter('map_type', MapType.KCITY)                       # "kcity" | "mirae"

        # 대회 타입에 따른 기본 모드 설정
        competition_type = str(self.get_parameter('competition_type').value)
        if competition_type == CompetitionType.QUALIFYING:
            self.declare_parameter('default_mode', ModeType.QUALIFYING_DRIVING)
        else:
            self.declare_parameter('default_mode', ModeType.FINAL_DRIVING)
        
        # 우회전 정지선 관련 파라미터
        self.declare_parameter('stopline_pause_distance', 5.0)                  # 정지선 감지 거리 (m)
        self.declare_parameter('enable_distance_condition', False)              # 거리 조건 사용 여부 on/off
        self.declare_parameter('stopline_pause_duration', 5.0)                  # 정지선 정지 시간 (s)
        self.declare_parameter('vehicle_stop_velocity_threshold', 0.01)         # 차량 정지 판단 속도 임계값 (m/s)

        # 신호등 전환 타이머 파라미터
        self.declare_parameter('traffic_signal_confirm_duration', 0.1)          # 신호등 전환 확인 시간 (s)
        
        # U턴 모드 제어 파라미터
        self.declare_parameter('use_uturn_flags', True)                         # U턴 플래그 사용 여부 (True: 플래그 기반, False: 구역 기반)
        
        # K-City 맵 구역 설정 (예선/본선 분리)
        self.declare_parameter('kcity_qualifying_parking_zones', [1, 2])        # 예선 주차 구역
        self.declare_parameter('kcity_uturn_zones', [7])                        # 예선 유턴 구역
        self.declare_parameter('kcity_gps_off_zones', [10])                     # 예선 GPS 차단 구역
        # 본선 배달 구역 (상차/하차 구분)
        self.declare_parameter('kcity_delivery_pickup_zones', [2])              # 본선 상차 구역 2번
        self.declare_parameter('kcity_delivery_dropoff_zones', [19])            # 본선 하차 구역
        self.declare_parameter('kcity_final_parking_zones', [21])               # 본선 주차 구역 21번
        self.declare_parameter('kcity_right_pause_zones', [13])                 # 본선 우회전 정지선 구역

        # 미래관 맵 구역 설정 (예선/본선 구분 없이 통일)
        self.declare_parameter('mirae_parking_zones', [7])                      # 주차 구역 
        self.declare_parameter('mirae_uturn_zones', [39])                       # 유턴 구역
        self.declare_parameter('mirae_gps_off_zones', [23])                     # GPS 차단 구역
        # 배달 구역 (상차/하차 구분)
        self.declare_parameter('mirae_delivery_pickup_zones', [28])             # 상차 구역
        self.declare_parameter('mirae_delivery_dropoff_zones', [28])            # 하차 구역
        self.declare_parameter('mirae_right_pause_zones', [55, 60])             # 우회전 정지선 구역

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
            
        # 정지선 정보 (우회전 정지선 처리용)
        self.stopline_distance_sub = self.create_subscription(
            Float64, '/stopline_distance', self._stopline_distance_cb, 10)
        self.stopline_type_sub = self.create_subscription(
            String, '/stopline_type', self._stopline_type_cb, 10)
            
        # 차량 위치 및 속도 정보 (정지 판단용)
        self.location_sub = self.create_subscription(
            Odometry, '/autocar/location', self._location_cb, 10)
        
        # 위치/맵 정보 (Lanelet ID를 Map ID로 변환)
        self.lanelet_id_sub = self.create_subscription(
            Int64, '/current_lanelet_id', self._lanelet_id_cb, 10)
        
        # 표지판 인식 (통합)
        self.target_sign_sub = self.create_subscription(
            Int32, '/target_sign', self._target_sign_cb, 10)
        
        # 미션 완료 플래그
        self.parking_completed_sub = self.create_subscription(
            Bool, '/parking_complete_flag', self._parking_completed_cb, 10)
        self.pickup_completed_sub = self.create_subscription(
            Bool, '/pickup_complete_flag', self._pickup_completed_cb, 10)
        self.delivery_completed_sub = self.create_subscription(
            Bool, '/delivery_complete_flag', self._delivery_completed_cb, 10)
        
        # U턴 플래그 (U턴 플래너에서 발행)
        self.uturn_start_sub = self.create_subscription(
            Bool, '/uturn_start_flag', self._uturn_start_cb, 10)
        self.uturn_complete_sub = self.create_subscription(
            Bool, '/uturn_complete_flag', self._uturn_complete_cb, 10)
        
        # 주차 포즈 계산 완료 플래그
        self.parking_pose_ready_sub = self.create_subscription(
            Bool, '/parking/pose_ready', self._parking_pose_ready_cb, 10)

        # ===========================================
        # 내부 상태 변수
        # ===========================================
        self.current_mode: str = str(self.get_parameter('default_mode').value)
        self.previous_mode: str = self.current_mode
        
        # 입력 상태들
        self.traffic_sign: Optional[str] = None                        # "Green", "Red", "Left", "Straightleft"
        self.current_lanelet_id: Optional[int] = None                  # 현재 차량이 위치한 Lanelet ID
        self.target_sign: Optional[int] = None                         # 1,2,3=상차 | 4,5,6=하차
        self.parking_complete_flag: bool = False
        self.pickup_complete_flag: bool = False                        # 상차 완료 플래그
        self.delivery_complete_flag: bool = False                      # 하차 완료 플래그
        self.parking_pose_ready: bool = False                          # 주차 포즈 계산 완료 플래그
        
        # U턴 플래그 (U턴 플래너에서 발행)
        self.uturn_start_flag: bool = False                            # U턴 시작 플래그
        self.uturn_complete_flag: bool = False                         # U턴 완료 플래그
        
        # 정지선 관련 상태
        self.stopline_distance: float = float('inf')                   # 정지선까지의 거리
        self.stopline_type: Optional[str] = None                       # 정지선 타입
        self.stopline_pause_start_time: Optional[float] = None         # 정지선 정지 시작 시간
        
        # 차량 속도 상태
        self.current_velocity: float = 0.0                             # 현재 차량 속도 (m/s)
        
        # 신호등 전환 타이머 상태
        self.traffic_signal_change_start_time: Optional[float] = None  # 신호등 변경 시작 시간
        self.pending_traffic_signal: Optional[str] = None              # 대기 중인 신호등 상태
        
        # 대회 및 맵 설정 가져오기
        self.competition_type: str = str(self.get_parameter('competition_type').value)
        self.map_type: str = str(self.get_parameter('map_type').value)
        
        # 현재 맵과 대회 타입에 따른 구역 ID 설정
        if self.map_type == MapType.KCITY:
            # K-City 맵 구역 설정
            if self.competition_type == CompetitionType.QUALIFYING:
                self.parking_zone_ids: List[int] = list(self.get_parameter('kcity_qualifying_parking_zones').value)
            else:  # FINAL
                self.parking_zone_ids: List[int] = list(self.get_parameter('kcity_final_parking_zones').value)
                
            self.uturn_zone_ids: List[int] = list(self.get_parameter('kcity_uturn_zones').value)
            self.gps_off_zone_ids: List[int] = list(self.get_parameter('kcity_gps_off_zones').value)
            self.delivery_pickup_zone_ids: List[int] = list(self.get_parameter('kcity_delivery_pickup_zones').value)
            self.delivery_dropoff_zone_ids: List[int] = list(self.get_parameter('kcity_delivery_dropoff_zones').value)
            self.right_pause_zone_ids: List[int] = list(self.get_parameter('kcity_right_pause_zones').value)
            
        else:  # MIRAE
            # 미래관 맵 구역 설정 (예선/본선 구분 없이 통일)
            self.parking_zone_ids: List[int] = list(self.get_parameter('mirae_parking_zones').value)
            self.uturn_zone_ids: List[int] = list(self.get_parameter('mirae_uturn_zones').value)
            self.gps_off_zone_ids: List[int] = list(self.get_parameter('mirae_gps_off_zones').value)
            self.delivery_pickup_zone_ids: List[int] = list(self.get_parameter('mirae_delivery_pickup_zones').value)
            self.delivery_dropoff_zone_ids: List[int] = list(self.get_parameter('mirae_delivery_dropoff_zones').value)
            self.right_pause_zone_ids: List[int] = list(self.get_parameter('mirae_right_pause_zones').value)
        
        # 미션 상태 플래그 (한 번만 수행되는 미션들)
        self.parking_mission_started: bool = False
        self.delivery_mission_started: bool = False
        
        # 모드 매핑 (String → ModeState 상수)
        self.mode_mapping = {
            # 예선 모드들
            ModeType.QUALIFYING_DRIVING: ModeState.DRIVE,
            ModeType.QUALIFYING_PARKING: ModeState.PARKING,
            ModeType.QUALIFYING_UTURN: ModeState.UTURN,        
            ModeType.QUALIFYING_GPS_OFF: ModeState.GPS_OFF,    
            
            # 본선 모드들
            ModeType.FINAL_DRIVING: ModeState.DRIVE,
            ModeType.FINAL_PARKING: ModeState.PARKING,
            ModeType.FINAL_PAUSE: ModeState.PAUSE,
            ModeType.FINAL_DELIVERY_PICKUP: ModeState.DELIVERY,
            ModeType.FINAL_DELIVERY_DROPOFF: ModeState.DELIVERY,
        }

        # ===========================================
        # 타이머 설정
        # ===========================================
        period = 1.0 / max(1e-3, float(self.get_parameter('publish_rate').value))
        self.create_timer(period, self._on_timer)

        self.get_logger().info(f'Mode Selector v2.0 started.')
        self.get_logger().info(f'Competition: {self.competition_type}, Map: {self.map_type}')
        self.get_logger().info(f'Parking zones ({self.competition_type}): {self.parking_zone_ids}')
        
        # U턴 모드 제어 방식 표시
        use_uturn_flags = bool(self.get_parameter('use_uturn_flags').value)
        if use_uturn_flags:
            self.get_logger().info('U턴 모드: 플래그 기반 (U턴 플래너 연동)')
        else:
            self.get_logger().info('U턴 모드: 구역 기반 (기존 방식)')
        
        if self.competition_type == CompetitionType.QUALIFYING:
            self.get_logger().info(f'Uturn zones: {self.uturn_zone_ids}')
            self.get_logger().info(f'GPS-off zones: {self.gps_off_zone_ids}')
        else:  # FINAL
            self.get_logger().info(f'Delivery zones: {self.delivery_pickup_zone_ids}')

    # ===========================================
    # 콜백 함수들
    # ===========================================
    def _traffic_sign_cb(self, msg: String) -> None:
        """교통 신호 수신"""
        signal = (msg.data or '').strip()
        if signal in ('Green', 'Red', 'Left', 'Straightleft'):
            self.traffic_sign = signal
            self.get_logger().debug(f'Traffic sign: {signal}')

    def _stopline_distance_cb(self, msg: Float64) -> None:
        """정지선 거리 수신"""
        try:
            self.stopline_distance = float(msg.data)
            self.get_logger().debug(f'Stopline distance: {self.stopline_distance:.2f}m')
        except Exception:
            self.stopline_distance = float('inf')

    def _stopline_type_cb(self, msg: String) -> None:
        """정지선 타입 수신"""
        stopline_type = (msg.data or '').strip()
        if stopline_type in ('no_stopline', 'nonstop', 'straight', 'right', 'left'):
            self.stopline_type = stopline_type
            self.get_logger().debug(f'Stopline type: {stopline_type}')
        else:
            self.stopline_type = 'no_stopline'

    def _location_cb(self, msg: Odometry) -> None:
        """차량 위치 및 속도 수신"""
        try:
            # 속력 계산 (절댓값)
            self.current_velocity = abs(msg.twist.twist.linear.x)
            self.get_logger().debug(f'Current velocity: {self.current_velocity:.2f} m/s')
        except Exception:
            self.current_velocity = 0.0

    def _lanelet_id_cb(self, msg: Int64) -> None:
        """현재 Lanelet ID 수신"""
        self.current_lanelet_id = int(msg.data)
        self.get_logger().debug(f'Current lanelet ID: {self.current_lanelet_id}')

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
            self.get_logger().info(f'주차 미션 완료! 현재 모드: {self.current_mode}')

    def _pickup_completed_cb(self, msg: Bool) -> None:
        """상차 미션 완료 플래그 수신"""
        if bool(msg.data):
            self.pickup_complete_flag = True
            self.get_logger().info('상차 미션 완료!')
    
    def _delivery_completed_cb(self, msg: Bool) -> None:
        """하차 미션 완료 플래그 수신"""
        if bool(msg.data):
            self.delivery_complete_flag = True
            self.get_logger().info('하차 미션 완료!')
            
    def _parking_pose_ready_cb(self, msg: Bool) -> None:
        """주차 포즈 계산 완료 플래그 수신"""
        if bool(msg.data):
            self.parking_pose_ready = True
            self.get_logger().info('주차 포즈 계산 완료!')
            
    def _uturn_start_cb(self, msg: Bool) -> None:
        """U턴 시작 플래그 수신"""
        if bool(msg.data):
            self.uturn_start_flag = True
            self.get_logger().info(f'U턴 시작 플래그 수신! 현재 모드: {self.current_mode}')
            
    def _uturn_complete_cb(self, msg: Bool) -> None:
        """U턴 완료 플래그 수신"""
        if bool(msg.data):
            self.uturn_complete_flag = True
            self.get_logger().info(f'U턴 완료 플래그 수신! 현재 모드: {self.current_mode}')

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
        """현재 상황을 종합하여 모드 결정 (예선/본선 구분)"""
        
        # ===========================================
        # 1. 미션 완료 처리 (최우선)
        # ===========================================
            
        # 본선 주차 완료 (강제 모드 전환)
        if self.parking_complete_flag and self.competition_type == CompetitionType.FINAL:
            self.get_logger().info(f'본선 주차 완료 - 강제 모드 전환: {self.current_mode} → FINAL_DRIVING')
            self.parking_complete_flag = False
            self.parking_mission_started = False
            return ModeType.FINAL_DRIVING
            
        # 예선 주차 완료 (강제 모드 전환)
        if self.parking_complete_flag and self.competition_type == CompetitionType.QUALIFYING:
            self.get_logger().info(f'예선 주차 완료 - 강제 모드 전환: {self.current_mode} → QUALIFYING_DRIVING')
            self.parking_complete_flag = False
            self.parking_mission_started = False
            return ModeType.QUALIFYING_DRIVING
            
        # 예선/본선에 따른 모드 결정 분기
        if self.competition_type == CompetitionType.QUALIFYING:
            return self._determine_qualifying_mode()
        else:  # FINAL
            return self._determine_final_mode()
    
    def _determine_qualifying_mode(self) -> str:
        """예선 모드 결정 로직"""
        
        # ===========================================
        # 예선 모드: QUALIFYING_DRIVING, QUALIFYING_PARKING, QUALIFYING_UTURN, QUALIFYING_GPS_OFF
        # ===========================================
        
        # 1. 주차 미션 처리
        #    (a) pose_ready 기반 전환: 플래너가 준비 신호를 보낸 경우 즉시 예선 주차 시작
        if (self.parking_pose_ready and
            not self.parking_mission_started and
            self.current_mode == ModeType.QUALIFYING_DRIVING):
            self.parking_mission_started = True
            self.parking_pose_ready = False
            self.get_logger().info('예선 주차 포즈 준비 완료 - QUALIFYING_PARKING 모드로 전환')
            return ModeType.QUALIFYING_PARKING

        #    (b) 구역 기반 전환: 지정된 예선 주차 구역에 진입했을 때 시작
        if (self.current_lanelet_id is not None and 
            self.current_lanelet_id in self.parking_zone_ids and 
            not self.parking_mission_started and
            self.current_mode == ModeType.QUALIFYING_DRIVING):
            
            self.parking_mission_started = True
            return ModeType.QUALIFYING_PARKING
        
        # 2. U턴 모드 처리 (플래그 기반 또는 구역 기반)
        use_uturn_flags = bool(self.get_parameter('use_uturn_flags').value)
        
        if use_uturn_flags:
            # 플래그 기반 U턴 모드 처리
            # U턴 완료 처리 (최우선)
            if self.uturn_complete_flag and self.current_mode == ModeType.QUALIFYING_UTURN:
                self.uturn_complete_flag = False
                self.get_logger().info('U턴 완료 - DRIVING 모드로 복귀')
                return ModeType.QUALIFYING_DRIVING
            
            # U턴 시작 처리
            if (self.uturn_start_flag and 
                self.current_mode == ModeType.QUALIFYING_DRIVING and
                self._is_in_uturn_zone()):
                
                self.uturn_start_flag = False
                self.get_logger().info('U턴 시작 - UTURN 모드로 전환')
                return ModeType.QUALIFYING_UTURN
                
        else:
            # 기존 구역 기반 U턴 모드 처리
            if (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.uturn_zone_ids):
                
                if self.current_mode == ModeType.QUALIFYING_DRIVING:
                    return ModeType.QUALIFYING_UTURN
                elif self.current_mode == ModeType.QUALIFYING_UTURN:
                    return ModeType.QUALIFYING_UTURN  # 구역 내에서 유턴 모드 유지
                
            # 유턴 구역을 벗어났을 때 DRIVING 모드로 복귀
            elif self.current_mode == ModeType.QUALIFYING_UTURN:
                return ModeType.QUALIFYING_DRIVING
            
        # 3. GPS 차단 구역 처리
        if (self.current_lanelet_id is not None and 
            self.current_lanelet_id in self.gps_off_zone_ids):
            
            if self.current_mode == ModeType.QUALIFYING_DRIVING:
                return ModeType.QUALIFYING_GPS_OFF
            elif self.current_mode == ModeType.QUALIFYING_GPS_OFF:
                return ModeType.QUALIFYING_GPS_OFF  # 구역 내에서 GPS_OFF 모드 유지
                
        # GPS 차단 구역을 벗어났을 때 DRIVING 모드로 복귀
        elif self.current_mode == ModeType.QUALIFYING_GPS_OFF:
            return ModeType.QUALIFYING_DRIVING
        
        # 4. 기본 주행 모드
        return ModeType.QUALIFYING_DRIVING
    
    def _determine_final_mode(self) -> str:
        """본선 모드 결정 로직"""

        # ===========================================
        # 본선 모드: FINAL_DRIVING, FINAL_PAUSE, FINAL_PARKING, FINAL_배달_상차, FINAL_배달_하차
        # ===========================================
        
        # 1. 우회전 정지선 처리 (최우선 - 신호등보다 상위)
        # 우회전 정지 구역을 벗어났는지 확인하여 타이머 초기화
        if not self._is_in_right_pause_zone():
            # 우회전 정지 구역을 벗어남 - 타이머 초기화
            if self.stopline_pause_start_time is not None:
                self.get_logger().info(f'우회전 정지 구역 이탈 - 타이머 초기화')
                self.stopline_pause_start_time = None
        
        # 우회전 정지선 조건 체크
        if self._should_pause_for_right_stopline():
            if self.current_mode == ModeType.FINAL_DRIVING:
                # 우회전 정지선 조건 만족 - PAUSE 모드 진입 (타이머는 아직 시작 안함)
                self.get_logger().info('우회전 정지선 감지 - PAUSE 모드 진입 (차량 정지 대기 중)')
                return ModeType.FINAL_PAUSE
        
        # 우회전 정지선 PAUSE 모드에서 실제 정지 확인 후 타이머 시작
        if (self.current_mode == ModeType.FINAL_PAUSE and 
            self._should_pause_for_right_stopline() and
            self.stopline_pause_start_time is None):
            
            # 차량이 실제로 정지했는지 확인
            velocity_threshold = float(self.get_parameter('vehicle_stop_velocity_threshold').value)
            
            if self.current_velocity <= velocity_threshold:
                # 실제 정지 확인 - 타이머 시작
                self.stopline_pause_start_time = self.get_clock().now().nanoseconds * 1e-9
                self.get_logger().info(f'차량 정지 확인 (속도: {self.current_velocity:.2f} m/s) - 3.2초 타이머 시작')
        
        # 우회전 정지선 정지 시간 완료 확인
        if (self.current_mode == ModeType.FINAL_PAUSE and 
            self.stopline_pause_start_time is not None):
            
            current_time = self.get_clock().now().nanoseconds * 1e-9
            pause_duration = float(self.get_parameter('stopline_pause_duration').value)
            
            if current_time - self.stopline_pause_start_time >= pause_duration:
                # 3.2초 대기 완료 - 자동 출발
                self.stopline_pause_start_time = None
                self.get_logger().info('우회전 정지선 대기 완료 - DRIVING 모드 복귀')
                return ModeType.FINAL_DRIVING
        
        # 2. 일반 신호등 처리 (우회전 정지선 구역이 아닐 때만)
        if not self._is_in_right_pause_zone():
            # Red 신호등 → PAUSE 모드 (즉시 전환)
            if self.traffic_sign == "Red" and self.current_mode == ModeType.FINAL_DRIVING:
                # 신호등 전환 타이머 리셋
                self.traffic_signal_change_start_time = None
                self.pending_traffic_signal = None
                return ModeType.FINAL_PAUSE
                
            # Green/Left/Straightleft 신호등 → DRIVING 모드 (0.3초 확인 후)
            if (self.traffic_sign in ("Green", "Left", "Straightleft") and 
                self.current_mode == ModeType.FINAL_PAUSE):
                
                # 신호등이 변경되었는지 확인
                if self.pending_traffic_signal != self.traffic_sign:
                    # 새로운 신호등 감지 - 타이머 시작
                    self.pending_traffic_signal = self.traffic_sign
                    self.traffic_signal_change_start_time = self.get_clock().now().nanoseconds * 1e-9
                    self.get_logger().info(f'신호등 변경 감지 ({self.traffic_sign}) - 0.3초 확인 대기')
                
                # 0.3초 지속 확인
                elif self.traffic_signal_change_start_time is not None:
                    current_time = self.get_clock().now().nanoseconds * 1e-9
                    confirm_duration = float(self.get_parameter('traffic_signal_confirm_duration').value)
                    
                    if current_time - self.traffic_signal_change_start_time >= confirm_duration:
                        # 0.3초 확인 완료 - DRIVING 모드 전환
                        self.traffic_signal_change_start_time = None
                        self.pending_traffic_signal = None
                        self.get_logger().info(f'신호등 확인 완료 ({self.traffic_sign}) - DRIVING 모드 전환')
                        return ModeType.FINAL_DRIVING

        # 2. 주차 미션 처리 (주차 포즈 계산 완료 시 시작)
        if (self.parking_pose_ready and 
            not self.parking_mission_started and
            self.current_mode == ModeType.FINAL_DRIVING):
            
            self.parking_mission_started = True
            self.parking_pose_ready = False  # 플래그 리셋
            self.get_logger().info('주차 포즈 계산 완료 - PARKING 모드로 전환')
            return ModeType.FINAL_PARKING

        # 3. 배달 미션 처리
        # 상차 모드 시작 조건 (target_sign 1,2,3 + 상차 구역)
        if (self.target_sign is not None and 
            1 <= self.target_sign <= 3 and                  # 상차 표지판
            not self.delivery_mission_started and           # 미션 시작 전
            self.current_mode == ModeType.FINAL_DRIVING and # 주행 중
            self._is_in_delivery_pickup_zone()):           # 상차 구역 내
            
            self.delivery_mission_started = True
            self.target_sign = None  # 플래그 리셋
            self.get_logger().info(f'상차 구역 {self.current_lanelet_id}에서 상차 표지판 {self.target_sign} 인식 - 상차 모드 시작')
            return ModeType.FINAL_DELIVERY_PICKUP

        # 상차 완료 조건 (PICKUP 모드에서 상차 완료 플래그 수신 시)
        if self.pickup_complete_flag and self.current_mode == ModeType.FINAL_DELIVERY_PICKUP:
            self.pickup_complete_flag = False  # 플래그 리셋
            self.get_logger().info('상차 완료 - DRIVING 모드로 복귀 (하차 구역으로 이동)')
            return ModeType.FINAL_DRIVING

        # 하차 모드 전환 조건 (target_sign 4,5,6 + 하차구역)
        if (self.current_mode == ModeType.FINAL_DRIVING and      # 주행 중
            self.delivery_mission_started and                    # 상차 완료 상태
            self._is_in_delivery_dropoff_zone() and             # 하차 구역 내
            self.target_sign is not None and
            4 <= self.target_sign <= 6):                        # 하차 표지판
            
            self.target_sign = None  # 플래그 리셋
            self.get_logger().info(f'하차 구역 {self.current_lanelet_id}에서 하차 표지판 {self.target_sign} 인식 - 하차 모드 시작')
            return ModeType.FINAL_DELIVERY_DROPOFF
            
        # 하차 완료 조건 (DROPOFF 모드에서 하차 완료 플래그 수신 시)
        if self.delivery_complete_flag and self.current_mode == ModeType.FINAL_DELIVERY_DROPOFF:
            self.delivery_complete_flag = False  # 플래그 리셋
            self.delivery_mission_started = False
            self.get_logger().info('하차 완료 - 배달 미션 종료, DRIVING 모드로 복귀')
            return ModeType.FINAL_DRIVING

        # 4. 현재 모드 유지
        return self.current_mode

    def _is_in_parking_zone(self) -> bool:
        """현재 주차 구역에 있는지 확인"""
        return (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.parking_zone_ids)

    def _is_in_delivery_pickup_zone(self) -> bool:
        """현재 배달 상차 구역에 있는지 확인"""
        return (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.delivery_pickup_zone_ids)
                
    def _is_in_delivery_dropoff_zone(self) -> bool:
        """현재 배달 하차 구역에 있는지 확인"""
        return (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.delivery_dropoff_zone_ids)
                
    def _is_in_uturn_zone(self) -> bool:
        """현재 유턴 구역에 있는지 확인"""
        return (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.uturn_zone_ids)
                
    def _is_in_gps_off_zone(self) -> bool:
        """현재 GPS 차단 구역에 있는지 확인"""
        return (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.gps_off_zone_ids)
                
    def _is_in_right_pause_zone(self) -> bool:
        """현재 우회전 정지선 구역에 있는지 확인"""
        return (self.current_lanelet_id is not None and 
                self.current_lanelet_id in self.right_pause_zone_ids)
                
    def _should_pause_for_right_stopline(self) -> bool:
        """우회전 정지선으로 인해 정지해야 하는지 확인"""
        # 우회전 정지선 구역에 있고
        if not self._is_in_right_pause_zone():
            return False
        
        # nonstop 정지선이면 정지하지 않음
        if self.stopline_type == "nonstop":
            return False
            
        # 정지선 타입이 "virtual"이고
        if self.stopline_type != "virtual":
            return False
            
        # 거리 조건 확인
        enable_distance = bool(self.get_parameter('enable_distance_condition').value)
        
        if enable_distance:
            # 거리 조건 ON: 정지선까지의 거리가 임계값 이내일 때만 정지
            distance_threshold = float(self.get_parameter('stopline_pause_distance').value)
            return self.stopline_distance <= distance_threshold
        else:
            # 거리 조건 OFF: 구역과 정지선 타입만 확인하고 거리 무관하게 정지
            return True
    

    def get_current_mode_info(self) -> dict:
        """현재 모드 상태 정보를 딕셔너리로 반환 (디버깅용)"""
        return {
            'current_mode': self.current_mode,
            'competition_type': self.competition_type,
            'map_type': self.map_type,
            'traffic_sign': self.traffic_sign,
            'lanelet_id': self.current_lanelet_id,
            'target_sign': self.target_sign,
            'parking_completed': self.parking_complete_flag,
            'pickup_completed': self.pickup_complete_flag,
            'delivery_completed': self.delivery_complete_flag,
            'uturn_start_flag': self.uturn_start_flag,
            'uturn_complete_flag': self.uturn_complete_flag,
            'stopline_distance': self.stopline_distance,
            'stopline_type': self.stopline_type,
            'stopline_pause_active': self.stopline_pause_start_time is not None,
            'current_velocity': self.current_velocity,
            'velocity_threshold': float(self.get_parameter('vehicle_stop_velocity_threshold').value),
            'traffic_signal_timer_active': self.traffic_signal_change_start_time is not None,
            'pending_traffic_signal': self.pending_traffic_signal,
            'traffic_signal_confirm_duration': float(self.get_parameter('traffic_signal_confirm_duration').value),
            'enable_distance_condition': bool(self.get_parameter('enable_distance_condition').value),
            'in_parking_zone': self._is_in_parking_zone(),
            'in_delivery_pickup_zone': self._is_in_delivery_pickup_zone(),
            'in_delivery_dropoff_zone': self._is_in_delivery_dropoff_zone(),
            'in_uturn_zone': self._is_in_uturn_zone(),
            'in_gps_off_zone': self._is_in_gps_off_zone(),
            'in_right_pause_zone': self._is_in_right_pause_zone(),
            'should_pause_for_stopline': self._should_pause_for_right_stopline(),
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