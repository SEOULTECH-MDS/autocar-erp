#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, PoseStamped, Pose, Quaternion, TransformStamped
from std_msgs.msg import ColorRGBA, Int32
from planning_msgs.msg import ObstacleArray, Obstacle, ConeMapMsg, ConeInfo
from builtin_interfaces.msg import Duration
import tf2_ros
import numpy as np
import math
from typing import List, Tuple, Optional



class ParkingArea:
    """주차 구역을 나타내는 클래스"""
    
    def __init__(self, area_id: int, boundary_points: List[Tuple[float, float]], 
                 cone_positions: List[Tuple[float, float]] = None):
        self.id = area_id
        self.boundary_points = boundary_points
        self.cone_positions = cone_positions or []
        self.center = self._calculate_center()
        self.is_open = False
    
    def _calculate_center(self) -> Tuple[float, float]:
        """구역의 중심점 계산"""
        if not self.boundary_points:
            return (0.0, 0.0)
        
        x_coords = [point[0] for point in self.boundary_points]
        y_coords = [point[1] for point in self.boundary_points]
        
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        
        return (center_x, center_y)


class VirtualWall:
    """가상 벽을 나타내는 클래스"""
    
    def __init__(self, start_point: Tuple[float, float], end_point: Tuple[float, float], 
                 width: float = 0.1):
        self.start_point = start_point
        self.end_point = end_point
        self.width = width  # 벽의 폭


class ParkingAreaNode(Node):
    """
    라바콘 위치 정보를 구독하여 주차 구역을 정의하고,
    열린 구역을 판단하며, 가상 벽을 생성하는 노드
    """
    
    def __init__(self):
        super().__init__('parking_area_node')
        
        # ──── ❶ 파라미터 선언 ───────────────────────────────────────────────
        self.declare_parameter('slot_origin_x', 0.0)      # X 기준선(도로 가장자리 x). 슬롯 좌우 경계와 라인 추정의 기준이 됩니다.
        self.declare_parameter('slot_origin_y', 0.0)      # 가장 아래 슬롯의 y 하단. 슬롯 y-밴드 경계의 기준이 됩니다.
        self.declare_parameter('SLOT_LEN', 5.0)           # 슬롯 세로 길이 L(기본 템플릿). 일렬 얼라인 시 관측 스팬으로 갱신됩니다.
        self.declare_parameter('SLOT_GAP', 0.01)           # 슬롯 간 간격 G. 슬롯 y-밴드 분리를 위한 여유 간격입니다.
        self.declare_parameter('SLOT_WIDTH', 2.5)         # 슬롯 가로 폭 W. 일렬 얼라인 사용 시에도 고정 유지됩니다.
        self.declare_parameter('EPS_X', 0.3)              # x 경계 여유 허용치. 경계 포함/누락 판정을 완화합니다.
        self.declare_parameter('EPS_Y', 0.01)              # y 경계 여유 허용치. 슬롯 y-밴드 판정을 완화합니다.
        # Recognition mode and sequential settings
        self.declare_parameter('recognition_mode', 'sequential')  # 인식 모드 선택(pattern|dynamic|sequential). 부분 관측 대응에는 sequential 권장.
        # 일렬 콘만으로 템플릿 얼라인 여부(폭은 고정 유지)
        self.declare_parameter('align.use_line_only', False)   # 도로측 일렬 콘만으로 템플릿 정렬(L만 갱신, W는 고정 유지). 활성화
        self.declare_parameter('align.min_span_y', 1.0)  # 일렬 관측 y-스팬 최소 보장값(너무 짧을 때 L 산정 하한).
        self.declare_parameter('align.lock_width', False)  # 관측 폭이 작거나 라인만 관측 시 W(폭) 고정 유지
        self.declare_parameter('align.min_width_span', 1.0)  # 폭 갱신을 허용하는 최소 x 스팬
        self.declare_parameter('align.disable', True)   # 템플릿 얼라인 활성화 (콘 위치에 맞춰 회전)
        # Sequential mode parameters
        # time/clock
        self.declare_parameter('seq.buffer_window_sec', 15.0)  # 최근 이 시간(초) 윈도의 콘만 버퍼링(부분 관측 누적 창 크기). (확대)
        self.declare_parameter('seq.cone_timeout_sec', 10000000000.0)   # 캐시된 콘의 유효 시간(초). 초과 시 제거합니다.
        self.declare_parameter('seq.confirm_threshold', 0.1)  # EMA 점수 확정 임계값. 높을수록 보수적으로 확정합니다.
        self.declare_parameter('seq.release_threshold', 0.9)  # EMA 점수 해제 임계값. 낮을수록 상태 유지가 강합니다.
        self.declare_parameter('seq.stable_hold_sec', 0.8)    # 확정 후 유지해야 하는 최소 시간(깜빡임 방지).
        self.declare_parameter('seq.ema_alpha', 0.5)          # EMA 가중치. 높을수록 최신 관측 반영이 큽니다.
        self.declare_parameter('seq.publish_stable_only', False)  # True면 안정 결과만 퍼블리시(실시간 변동 억제).
        self.declare_parameter('seq.logging_enabled', False)       # 시퀀스 로그 기록 on/off.
        self.declare_parameter('seq.log_path', '/tmp/parking_area_seq.csv')  # 시퀀스 로그 파일 경로.
        self.declare_parameter('seq.map_cache_ttl_sec', 30.0)     # 라벨/맵 캐시 TTL(초). 오래된 항목을 자동 삭제합니다. (확대)
        # Robustness controls against frame drops / oscillation
        self.declare_parameter('seq.skip_on_drop', False)      # 버퍼 크기 급감 시 업데이트 건너뜀(진동 억제).
        self.declare_parameter('seq.min_score_delta', 0.08)    # 모드 전환 최소 점수 차이. 클수록 전환 둔감.
        self.declare_parameter('seq.provisional_dwell_sec', 0.4)  # 임시 우승 모드 유지 필요 시간(전환 안정화).
        self.declare_parameter('seq.release_grace_sec', 0.8)      # 해제 조건 지속 필요 시간(불안정 기간 허용).
        # Road-side line extraction params
        self.declare_parameter('line.quantile', 0.2)           # 일렬 대표 x 추정 시 좌측 분위수(도로측 선택 정도).
        self.declare_parameter('line.x_tolerance', 0.35)       # 대표 x 주변 허용 오차(m). 일렬 포인트 판정 폭.
        self.declare_parameter('line.min_points', 2)           # 일렬로 인정할 최소 포인트 수.
        # PCA/RANSAC line fit params
        self.declare_parameter('line.use_pca', False)           # PCA로 주성분(도로측 라인) 추정 사용
        self.declare_parameter('line.distance_tolerance', 0.5)  # 라인으로부터 허용 거리(일렬로 채택 기준) - 완화
        self.declare_parameter('line.use_ransac', False)       # RANSAC 사용 여부(옵션)
        # Gap-based open-slot detection
        self.declare_parameter('gap.use', False)                # s-축 연속 점들 간 큰 간격을 열린 구역으로 판정
        self.declare_parameter('gap.min_length', 3.0)        # 열린 구역으로 간주할 최소 갭 길이(m) - 사용자 제안 반영
        self.declare_parameter('gap.max_cone_count', 4)       # 갭 계산에 사용할 최대 콘 개수
        self.declare_parameter('gap.method', 'distance_range')  # 'closest_cone' | 'distance_range'
        self.declare_parameter('gap.distance_range_min', 4.0) # 방법2: 최소 콘간 거리(m)
        self.declare_parameter('gap.distance_range_max', 6.0) # 방법2: 최대 콘간 거리(m)
        self.declare_parameter('gap.pick_strategy', 'largest') # largest | first
        # Simple mode (경량 판정 전용 경로)
        self.declare_parameter('simple.enabled', False)        # True면 단순 파이프라인만 수행
        self.declare_parameter('simple.side', 'right')         # right|left|all
        self.declare_parameter('simple.gap_min', 2.5)          # 갭 기반 열린 슬롯 임계(m)
        self.declare_parameter('simple.width', 2.5)            # 슬롯 폭 가정(중심 오프셋 계산용)
        self.declare_parameter('simple.distance_tolerance', 0.5)  # 라인 법선 허용 거리
        # Hardcoded noise filtering (하드코딩 노이즈 필터링)
        self.declare_parameter('hardcoded_filter.enable', True)   # 하드코딩 필터링 활성화
        self.declare_parameter('hardcoded_filter.y_min', 0.0)    # 주차 구역 최소 Y 좌표 (map frame)
        self.declare_parameter('hardcoded_filter.y_max', 40.0)    # 주차 구역 최대 Y 좌표 (map frame)
        self.declare_parameter('hardcoded_filter.x_min', -10.0)     # 주차 구역 최소 X 좌표 (map frame)
        self.declare_parameter('hardcoded_filter.x_max', 20.0)    # 주차 구역 최대 X 좌표 (map frame)
        # Pattern matching (패턴 매칭)
        self.declare_parameter('pattern.enable', False)            # 패턴 매칭 활성화
        self.declare_parameter('pattern.min_cones', 4)            # 패턴 매칭 최소 콘 개수
        self.declare_parameter('pattern.score_threshold', 0.3)    # 패턴 매칭 점수 임계값
        # Persistent cone map (미션 종료까지 보존)
        self.declare_parameter('persist.enable', False)          # 영속 맵 사용 여부
        self.declare_parameter('persist.merge_radius', 0.3)     # 같은 콘으로 병합하는 반경(m)
        self.declare_parameter('persist.use_for_decision', False)  # 의사결정에 영속 맵 우선 사용 - 부분 관측 대응
        self.declare_parameter('persist.freeze_after_confirm', False) # 안정 확정 후 갱신 정지
        self.declare_parameter('persist.reset_topic', '/parking/reset_map') # 맵 리셋 토픽(Empty)
        # 디버그/시각화 파라미터
        self.declare_parameter('debug.publish_line_cones', False)    # 도로측 라인 콘 시각화 퍼블리시
        self.declare_parameter('debug.line_cones_topic', '/debug/line_cones')  # 라인 콘 시각화 토픽
        # 노이즈 필터링 파라미터
        self.declare_parameter('filter.enable_noise_removal', False) # 노이즈 콘 제거 활성화
        self.declare_parameter('filter.max_distance_from_vehicle', 15.0)  # 차량으로부터 최대 거리 (m)
        self.declare_parameter('filter.min_cluster_size', 2)        # 클러스터 최소 크기
        self.declare_parameter('filter.cluster_radius', 1.5)       # 클러스터링 반경 (m)
        
        # 통계적 안정성 파라미터 (새로운 접근 방식)
        self.declare_parameter('statistical.enable', True)        # 통계적 안정성 방식 활성화
        self.declare_parameter('statistical.min_frames', 5)        # 최소 프레임 수
        self.declare_parameter('statistical.position_tolerance', 0.5)  # 위치 허용 오차 (m)
        self.declare_parameter('statistical.confidence_threshold', 0.7)  # 신뢰도 임계값
        self.declare_parameter('statistical.max_history', 20)      # 최대 히스토리 길이
        
        # 차량 궤적 파라미터
        self.declare_parameter('trajectory.enable', False)         # 차량 궤적 분석 활성화
        self.declare_parameter('trajectory.lookback_distance', 10.0)  # 궤적 분석 거리 (m)
        self.declare_parameter('trajectory.perpendicular_tolerance', 0.3)  # 수직 허용 오차 (rad)
        self.declare_parameter('trajectory.min_cone_distance', 1.0)  # 최소 콘 거리 (m)
        
        # 파라미터 로드
        self.slot_origin_x = self.get_parameter('slot_origin_x').value
        self.slot_origin_y = self.get_parameter('slot_origin_y').value
        self.SLOT_LEN = self.get_parameter('SLOT_LEN').value
        self.SLOT_GAP = self.get_parameter('SLOT_GAP').value
        self.SLOT_WIDTH = self.get_parameter('SLOT_WIDTH').value
        self.EPS_X = self.get_parameter('EPS_X').value
        self.EPS_Y = self.get_parameter('EPS_Y').value
        
        # 갭 파라미터 로드 (통계적 방식에서도 사용)
        try:
            self.gap_min_length = float(self.get_parameter('gap.min_length').value)
        except Exception:
            self.gap_min_length = 3.0
        
        # 통계적 안정성 파라미터 로드
        self.statistical_enable = self.get_parameter('statistical.enable').value
        self.statistical_min_frames = self.get_parameter('statistical.min_frames').value
        self.statistical_position_tolerance = self.get_parameter('statistical.position_tolerance').value
        self.statistical_confidence_threshold = self.get_parameter('statistical.confidence_threshold').value
        self.statistical_max_history = self.get_parameter('statistical.max_history').value
        
        # 차량 궤적 파라미터 로드
        self.trajectory_enable = self.get_parameter('trajectory.enable').value
        self.trajectory_lookback_distance = self.get_parameter('trajectory.lookback_distance').value
        self.trajectory_perpendicular_tolerance = self.get_parameter('trajectory.perpendicular_tolerance').value
        self.trajectory_min_cone_distance = self.get_parameter('trajectory.min_cone_distance').value
        
        # 구독자 - 콘 맵 기반으로 변경
        self.cone_map_sub = self.create_subscription(
            ConeMapMsg,
            '/cone_map',
            self.process_cone_map,
            10
        )
        
        # 차량 위치 구독자 (궤적 분석용)
        self.vehicle_pose_sub = self.create_subscription(
            PoseStamped,
            '/vehicle_pose',
            self.process_vehicle_pose,
            10
        )
        
        # 퍼블리셔
        self.open_area_pub = self.create_publisher(
            Int32,
            '/open_parking_area_id',
            10
        )
        
        self.virtual_walls_pub = self.create_publisher(
            ObstacleArray,
            '/virtual_walls',
            10
        )
        
        # Rviz 시각화 퍼블리셔
        self.visualization_pub = self.create_publisher(
            MarkerArray,
            '/parking_area_visualization',
            10
        )
        
        # 플래너용 열린 슬롯 중심점 퍼블리셔 추가
        self.open_slot_pose_pub = self.create_publisher(
            PoseStamped,
            '/open_slot_pose',
            10
        )
        # Debug visualization publisher (line cones)
        self.line_cones_debug_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('debug.line_cones_topic').value),
            10
        )
        
        # Stable topic publishers (sequential mode)
        self.open_slot_pose_stable_pub = self.create_publisher(
            PoseStamped,
            '/open_slot_pose_stable',
            10
        )
        self.virtual_walls_stable_pub = self.create_publisher(
            ObstacleArray,
            '/virtual_walls_stable',
            10
        )
        
        # TF 브로드캐스터 추가
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # ──── 패턴 데이터 정의 (cones_node.py와 동기화) ───────────────────────────────
        # 3가지 경우의 수에 따른 예상 콘 위치 패턴 (상대 좌표)
        self.LAYOUT_PATTERNS = {
            "pattern_area_2_open": {
                "open_slot": 2,  # 위쪽 열림
                "closed_slots": [0, 1],
                "expected_cone_positions": [
                    (0, 0), (0, 1.25), (0, 2.5), (0, 3.75), (0, 5), (0, 6.25), (0, 7.5), (0, 8.75), (0, 10), (0, 15),
                    (1.25, 0), (1.25, 5), (1.25, 10), (1.25, 15),
                    (2.5, 0), (2.5, 2.5), (2.5, 5), (2.5, 7.5), (2.5, 10), (2.5, 12.5), (2.5, 15)
                ]
            },
            "pattern_area_1_open": {
                "open_slot": 1,  # 중간 열림
                "closed_slots": [0, 2],
                "expected_cone_positions": [
                    (0, 0), (0, 1.25), (0, 2.5), (0, 3.75), (0, 5), (0, 10), (0, 11.25), (0, 12.5), (0, 13.75), (0, 15),
                    (1.25, 0), (1.25, 5), (1.25, 10), (1.25, 15),
                    (2.5, 0), (2.5, 2.5), (2.5, 5), (2.5, 7.5), (2.5, 10), (2.5, 12.5), (2.5, 15)
                ]
            },
            "pattern_area_0_open": {
                "open_slot": 0,  # 아래쪽 열림
                "closed_slots": [1, 2],
                "expected_cone_positions": [
                    (0, 0), (0, 5), (0, 6.25), (0, 7.5), (0, 8.75), (0, 10), (0, 11.25), (0, 12.5), (0, 13.75), (0, 15),
                    (1.25, 0), (1.25, 5), (1.25, 10), (1.25, 15),
                    (2.5, 0), (2.5, 2.5), (2.5, 5), (2.5, 7.5), (2.5, 10), (2.5, 12.5), (2.5, 15)
                ]
            }
        }
        
        # 패턴 매칭 임계값
        self.PATTERN_MATCH_THRESHOLD = 0.7  # 70% 이상 일치
        self.CONE_MATCH_TOLERANCE = 0.3     # 30cm 허용 오차
        
        # 구역 정보 초기화
        self.areas = []
        self.open_area_id = 0
        self.virtual_walls = []
        self.detected_pattern = None
        # Sequential mode state
        self._seq_cone_cache = {}  # cone_id -> {'x':float,'y':float,'t':float}
        self._seq_ema_scores = {k: 0.0 for k in self.LAYOUT_PATTERNS.keys()}
        self._seq_stable_candidate = None
        self._seq_stable_since = None
        self._seq_stable_pattern = None
        self._seq_log_file = None
        self._seq_prev_buffer_count = 0
        self._seq_last_inst_scores = None
        self._seq_prov_candidate = None
        self._seq_prov_since = None
        self._seq_release_below_since = None
        # Persistent cones (TTL 없음, 리셋 전까지 보존)
        self._persist_cones = []   # [{'x':..,'y':..}]
        self._persist_frozen = False
        from std_msgs.msg import Empty
        self.create_subscription(Empty, str(self.get_parameter('persist.reset_topic').value), self._on_reset_persist, 10)
        
        # 통계적 안정성을 위한 데이터 구조
        self._cone_history = {}  # cone_id -> [{'x':.., 'y':.., 't':.., 'confidence':..}]
        self._stable_cones = {}  # cone_id -> {'x':.., 'y':.., 'confidence':.., 'last_seen':..}
        self._vehicle_poses = []  # 차량 궤적 히스토리 [{'x':.., 'y':.., 'yaw':.., 't':..}]
        self._current_vehicle_pose = None  # 현재 차량 위치
        
        # 타이머 설정 (10Hz로 시각화 업데이트)
        self.timer = self.create_timer(0.1, self.publish_visualization)
        
        self.get_logger().info('Parking Area Node가 시작되었습니다.')
    # ──── 패턴 매칭 알고리즘 ───────────────────────────────────────────────
    def detect_layout_pattern(self, cone_positions: List[Tuple[float, float]]) -> str:
        """실제 콘 위치를 3가지 패턴과 비교하여 매칭"""
        if len(cone_positions) < 10:  # 최소 콘 개수 확인
            self.get_logger().info(f'패턴 매칭용 콘 개수 부족 ({len(cone_positions)}/10)')
            return None
        
        best_pattern = None
        best_score = -1
        
        for pattern_name, pattern_data in self.LAYOUT_PATTERNS.items():
            score = self.calculate_pattern_match_score(cone_positions, pattern_data)
            
            self.get_logger().debug(f'{pattern_name}: 매칭 점수 {score:.3f}')
            
            if score > best_score:
                best_score = score
                best_pattern = pattern_name
        
        # 임계값 이상일 때만 패턴 확정
        if best_score > self.PATTERN_MATCH_THRESHOLD:
            self.get_logger().info(f'🎯 패턴 매칭 성공: {best_pattern} (점수: {best_score:.3f})')
            return best_pattern
        else:
            self.get_logger().warn(f'⚠️ 패턴 매칭 실패 (최고점수: {best_score:.3f})')
            return None
    
    def calculate_pattern_match_score(self, actual_cones: List[Tuple[float, float]], pattern_data: dict) -> float:
        """패턴과 실제 콘의 일치도 계산"""
        expected_cones = pattern_data["expected_cone_positions"]
        
        # 기준점 자동 정렬 (실제 콘의 최소점을 0,0으로)
        if not actual_cones:
            return 0.0
        
        min_x = min(pos[0] for pos in actual_cones)
        min_y = min(pos[1] for pos in actual_cones)
        normalized_actual = [(x - min_x, y - min_y) for x, y in actual_cones]
        
        # 매칭 점수 계산
        matches = 0
        
        for expected_pos in expected_cones:
            for actual_pos in normalized_actual:
                distance = np.sqrt((expected_pos[0] - actual_pos[0])**2 + 
                                 (expected_pos[1] - actual_pos[1])**2)
                if distance < self.CONE_MATCH_TOLERANCE:
                    matches += 1
                    break
        
        return matches / len(expected_cones) if expected_cones else 0.0
    
    def normalize_cone_positions(self, cone_positions: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """콘 위치를 정규화 (최소점을 0,0으로)"""
        if not cone_positions:
            return []
        
        min_x = min(pos[0] for pos in cone_positions)
        min_y = min(pos[1] for pos in cone_positions)
        
        return [(x - min_x, y - min_y) for x, y in cone_positions]
    
    def adjust_parameters_to_actual_cones(self, cone_positions: List[Tuple[float, float]]):
        """실제 콘 위치로 기준점 미세 조정"""
        if not cone_positions:
            return
        
        min_x = min(pos[0] for pos in cone_positions)
        min_y = min(pos[1] for pos in cone_positions)
        max_x = max(pos[0] for pos in cone_positions)
        max_y = max(pos[1] for pos in cone_positions)
        
        # 기준점 조정
        old_origin_x = self.slot_origin_x
        old_origin_y = self.slot_origin_y
        
        self.slot_origin_x = min_x
        self.slot_origin_y = min_y
        
        # 크기 정보 업데이트 (폭 잠금/최소 스팬 보호)
        x_span = max_x - min_x
        if (not bool(self.get_parameter('align.lock_width').value)) and (x_span >= float(self.get_parameter('align.min_width_span').value)):
            self.SLOT_WIDTH = x_span
        total_length = max_y - min_y
        self.SLOT_LEN = total_length / 3.0
        
        # 변경사항 로그
        if (abs(old_origin_x - self.slot_origin_x) > 0.1 or 
            abs(old_origin_y - self.slot_origin_y) > 0.1):
            self.get_logger().info('📍 실제 콘 위치로 파라미터 조정:')
            self.get_logger().info(f'  기준점: ({old_origin_x:.2f}, {old_origin_y:.2f}) → ({self.slot_origin_x:.2f}, {self.slot_origin_y:.2f})')
            self.get_logger().info(f'  크기: {self.SLOT_WIDTH:.2f} × {total_length:.2f}')

    def adjust_parameters_to_line(self, line_cones: List[Tuple[float, float]]):
        """도로측 일렬 콘만으로 슬롯 템플릿을 얼라인
        - SLOT_WIDTH는 기존 파라미터를 유지(가로 폭은 고정)
        - SLOT_LEN은 관측된 y 범위를 3등분해서 설정(최소 스팬 보장)
        - slot_origin_x는 일렬 콘의 대표 x(중앙값 근사: 평균)로 설정
        - slot_origin_y는 관측 y의 최솟값으로 설정
        """
        if not line_cones:
            return
        ys = [p[1] for p in line_cones]
        xs = [p[0] for p in line_cones]
        min_y, max_y = min(ys), max(ys)
        span_y = max(max_y - min_y, float(self.get_parameter('align.min_span_y').value))
        rep_x = sum(xs) / float(len(xs))

        old_origin_x = self.slot_origin_x
        old_origin_y = self.slot_origin_y

        self.slot_origin_x = rep_x
        self.slot_origin_y = min_y
        self.SLOT_LEN = span_y / 3.0

        if (abs(old_origin_x - self.slot_origin_x) > 0.1 or 
            abs(old_origin_y - self.slot_origin_y) > 0.1):
            self.get_logger().info('📍 일렬 콘 기준 템플릿 얼라인:')
            self.get_logger().info(f'  기준점: ({old_origin_x:.2f}, {old_origin_y:.2f}) → ({self.slot_origin_x:.2f}, {self.slot_origin_y:.2f})')
            self.get_logger().info(f'  길이(L): {self.SLOT_LEN*3.0:.2f} (슬롯당 {self.SLOT_LEN:.2f}), 폭(W 고정): {self.SLOT_WIDTH:.2f}')
    
    def process_cone_map(self, cone_map: ConeMapMsg):
        """콘 맵 데이터를 ObstacleArray로 변환하여 기존 로직 활용"""
        if not cone_map.coordinate_system_ready:
            self.get_logger().warn("⚠️ 콘 맵 좌표계가 아직 준비되지 않음")
            return
        
        # 좌표계 정보 저장 (역변환을 위해)
        self._cone_map_origin = (cone_map.origin.x, cone_map.origin.y)
        self._cone_map_rotation = cone_map.rotation_angle
        
        # 신뢰도 높은 콘만 필터링
        reliable_cones = [cone for cone in cone_map.cones if cone.confidence >= 1.5]
        
        # 🔧 하드코딩 필터링: 노이즈 콘 제거 (원본 map 좌표 기준)
        if self.get_parameter('hardcoded_filter.enable').get_parameter_value().bool_value:
            y_min = self.get_parameter('hardcoded_filter.y_min').get_parameter_value().double_value
            y_max = self.get_parameter('hardcoded_filter.y_max').get_parameter_value().double_value
            x_min = self.get_parameter('hardcoded_filter.x_min').get_parameter_value().double_value
            x_max = self.get_parameter('hardcoded_filter.x_max').get_parameter_value().double_value
            
            filtered_cones = []
            noise_removed_count = 0
            for cone in reliable_cones:
                # 원본 map 좌표의 영역 확인
                x, y = cone.original_position.x, cone.original_position.y
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    filtered_cones.append(cone)
                else:
                    noise_removed_count += 1
                    self.get_logger().debug(f"🗑️ 노이즈 콘 제거: ID={cone.id}, 위치=({x:.2f}, {y:.2f})")
            
            self.get_logger().info(f"🧹 하드코딩 필터링: {noise_removed_count}개 제거, {len(filtered_cones)}개 유지")
            self.get_logger().info(f"   필터 영역: X=[{x_min:.1f}, {x_max:.1f}], Y=[{y_min:.1f}, {y_max:.1f}]")
            reliable_cones = filtered_cones
        
        self.get_logger().info(f"🗺️ 콘 맵 수신: 총 {len(cone_map.cones)}개, 신뢰도 높은 콘: {len(reliable_cones)}개")
        self.get_logger().info(f"   좌표계: 원점=({cone_map.origin.x:.2f}, {cone_map.origin.y:.2f}), 회전={math.degrees(cone_map.rotation_angle):.1f}°")
        
        if len(reliable_cones) == 0:
            self.get_logger().warn("⚠️ 신뢰도 높은 콘이 없습니다!")
            return
        
        # ConeMapMsg를 ObstacleArray로 변환 (기존 로직 재사용을 위해)
        obstacle_array = ObstacleArray()
        obstacle_array.header = cone_map.header
        obstacle_array.header.frame_id = 'cone_map_normalized'  # 정규화된 좌표계 표시
        
        for cone in reliable_cones:
            obstacle = Obstacle()
            obstacle.id = cone.id
            
            # 정규화된 좌표계가 준비되었으면 정규화된 좌표 사용, 아니면 원본 좌표 사용
            if cone_map.coordinate_system_ready:
                obstacle.center.x = cone.position.x
                obstacle.center.y = cone.position.y
                # 디버깅: 좌표 변환 로그
                if abs(cone.position.x) > 0.01 or abs(cone.position.y) > 0.01:
                    self.get_logger().debug(f"정규화 좌표 사용 - 콘[{cone.id}]: 정규화=({cone.position.x:.2f}, {cone.position.y:.2f}), 원본=({cone.original_position.x:.2f}, {cone.original_position.y:.2f})")
            else:
                obstacle.center.x = cone.original_position.x
                obstacle.center.y = cone.original_position.y
                self.get_logger().debug(f"원본 좌표 사용 - 콘[{cone.id}]: ({cone.original_position.x:.2f}, {cone.original_position.y:.2f})")
            
            obstacle.center.z = 0.0
            
            # 메타데이터 추가 (기존 코드와 호환되도록 side_label만 사용)
            obstacle.description = cone.side_label
            
            obstacle_array.obstacles.append(obstacle)
        
        # 기존 process_cones 로직 호출
        self.process_cones(obstacle_array)
    
    def process_cones(self, cone_obstacles: ObstacleArray):
        """라바콘 위치 정보를 처리하여 구역을 정의하고 분석"""
        
        # 1단계: 입력 데이터 확인 로그
        self.get_logger().info(f"🔍 콘 데이터 수신: {len(cone_obstacles.obstacles)}개 콘")
        self.get_logger().info(f"   프레임: {cone_obstacles.header.frame_id}")
        self.get_logger().info(f"   타임스탬프: {cone_obstacles.header.stamp.sec}.{cone_obstacles.header.stamp.nanosec}")
        
        # 각 콘 정보 출력 (최대 5개)
        for i, cone in enumerate(cone_obstacles.obstacles[:5]):
            desc = getattr(cone, 'description', 'none')
            self.get_logger().info(f"   콘[{i}]: id={cone.id} pos=({cone.center.x:.2f}, {cone.center.y:.2f}) desc='{desc}'")
        
        if len(cone_obstacles.obstacles) == 0:
            self.get_logger().warn("⚠️ 콘 데이터가 비어있습니다!")
            return
        
        # 경량 모드: 가장 단순한 라인+갭 기반 판정만 수행
        if bool(self.get_parameter('simple.enabled').value):
            self.get_logger().info("🎯 단순 모드로 처리 중...")
            try:
                self.process_cones_simple(cone_obstacles)
            except Exception as e:
                self.get_logger().error(f"단순 모드 처리 오류: {e}")
            return
        
        mode = str(self.get_parameter('recognition_mode').value).lower()
        self.get_logger().info(f"📋 인식 모드: {mode}")
        if mode == 'sequential':
            return self.process_cones_sequential(cone_obstacles)

        # 라바콘 위치 추출 (pattern/dynamic 기존 방식)
        # 차량 기준 오른쪽 콘만 사용 (어댑터에서 description='left/right/center' 제공 시)
        cone_positions = []
        for obs in cone_obstacles.obstacles:
            try:
                if hasattr(obs, 'description') and obs.description:
                    if str(obs.description).strip().lower() != 'right':
                        continue
            except Exception:
                pass
            cone_positions.append((obs.center.x, obs.center.y))
        
        # ──── ❷ 패턴 매칭 기반 콘 콜백 처리 ───────────────────────────────────────────────
        # STEP 1: 패턴 매칭 시도
        detected_pattern = self.detect_layout_pattern(cone_positions)
        
        if detected_pattern is None:
            # 패턴 매칭 실패 시 기존 동적 로직 사용
            self.get_logger().info('패턴 매칭 실패, 동적 분석 모드로 전환')
            return self.process_cones_dynamic_fallback(cone_positions)
        
        # STEP 2: 패턴에 따른 구역 설정
        pattern_data = self.LAYOUT_PATTERNS[detected_pattern]
        self.open_area_id = pattern_data["open_slot"]
        self.detected_pattern = detected_pattern
        
        # STEP 3: 실제 콘 위치로 파라미터 미세 조정
        self.adjust_parameters_to_actual_cones(cone_positions)
        
        # STEP 4: 구역 생성 (패턴 기반)
        self.areas = self.build_parking_areas(cone_positions)
        
        # STEP 5: 각 구역의 열림/닫힘 상태 설정
        for i, area in enumerate(self.areas):
            area.is_open = (i == self.open_area_id)
        
        # STEP 6: 가상 벽 생성
        self.virtual_walls = []
        
        # 열린 구역: ㄷ-자 벽 생성 (차가 들어갈 수 있도록)
        if self.open_area_id >= 0 and self.open_area_id < len(self.areas):
            open_walls = self.make_D_shaped_walls(self.areas[self.open_area_id])
            self.virtual_walls.extend(open_walls)
        
        # 닫힌 구역들: 완전한 사각형 벽으로 둘러싸기
        for i, area in enumerate(self.areas):
            if not area.is_open:  # 닫힌 구역만
                closed_walls = self.make_rectangular_walls(area)
                self.virtual_walls.extend(closed_walls)
        
        # STEP 7: 결과 퍼블리시
        self.publish_results()

    def process_cones_simple(self, cone_obstacles: ObstacleArray) -> None:
        """단순 파이프라인: 우측(또는 지정) 콘만으로 PCA 라인→s-갭으로 열린 슬롯 판정 후 포즈만 퍼블리시"""
        # 1) 입력 콘 수집 + 측면 선택
        side = str(self.get_parameter('simple.side').value).lower().strip()
        self.get_logger().info(f"🎯 단순 모드: {side} 측면 콘 선택")
        
        cones = []
        filtered_count = 0
        for obs in cone_obstacles.obstacles:
            desc = (obs.description or '').lower().strip() if hasattr(obs, 'description') else ''
            self.get_logger().info(f"   콘 검사: pos=({obs.center.x:.2f}, {obs.center.y:.2f}) desc='{desc}' side='{side}'")
            
            if side == 'right' and desc and desc != 'right':
                self.get_logger().info(f"      → 필터됨 (right 모드인데 desc='{desc}')")
                filtered_count += 1
                continue
            if side == 'left' and desc and desc != 'left':
                self.get_logger().info(f"      → 필터됨 (left 모드인데 desc='{desc}')")
                filtered_count += 1
                continue
            
            self.get_logger().info(f"      → 선택됨")
            cones.append((float(obs.center.x), float(obs.center.y)))
        
        self.get_logger().info(f"📊 선택된 콘: {len(cones)}개, 필터된 콘: {filtered_count}개")
        if len(cones) < 2:
            return
        xs = np.array([p[0] for p in cones], dtype=float)
        ys = np.array([p[1] for p in cones], dtype=float)
        cx, cy = float(np.mean(xs)), float(np.mean(ys))
        X = np.stack([xs - cx, ys - cy], axis=0)
        C = np.cov(X)
        eigvals, eigvecs = np.linalg.eig(C)
        v = eigvecs[:, int(np.argmax(eigvals))]  # 진행방향(s)
        n = np.array([-v[1], v[0]])
        s = v[0] * (xs - cx) + v[1] * (ys - cy)
        d = n[0] * (xs - cx) + n[1] * (ys - cy)
        tol = float(self.get_parameter('simple.distance_tolerance').value)
        mask = np.abs(d) <= tol
        if not np.any(mask):
            return
        s_in = np.sort(s[mask])
        if s_in.size < 2:
            return
        diffs = np.diff(s_in)
        gap_min = float(self.get_parameter('simple.gap_min').value)
        idx = np.where(diffs >= gap_min)[0]
        if idx.size == 0:
            return
        gi = int(idx[np.argmax(diffs[idx])])  # 가장 큰 갭 선택
        s_mid = float(0.5 * (s_in[gi] + s_in[gi + 1]))
        # 2) 슬롯 ID 추정(3등분)
        s_min, s_max = float(np.min(s_in)), float(np.max(s_in))
        span = max(1e-6, s_max - s_min)
        band = span / 3.0
        sid = int(np.clip(np.floor((s_mid - s_min) / band), 0, 2))
        # 3) 포즈 계산: 라인 중앙(s_mid)에서 슬롯 내부로 폭/2 만큼 법선 방향 이동
        px_line = cx + v[0] * s_mid
        py_line = cy + v[1] * s_mid
        width = float(self.get_parameter('simple.width').value)
        px = px_line + n[0] * (width * 0.5)
        py = py_line + n[1] * (width * 0.5)
        yaw = math.atan2(v[1], v[0])
        # 4) 퍼블리시: open_area_id + pose만
        self.open_area_id = sid
        # 열린 구역 ID 퍼블리시
        area_msg = Int32()
        area_msg.data = sid
        self.open_area_pub.publish(area_msg)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(px)
        pose.pose.position.y = float(py)
        pose.pose.position.z = 0.0
        q = Quaternion()
        q.x = 0.0; q.y = 0.0
        q.z = float(np.sin(yaw * 0.5))
        q.w = float(np.cos(yaw * 0.5))
        pose.pose.orientation = q
        self.open_slot_pose_pub.publish(pose)

    def _seq_log(self, t: float, cone_id: int, x: float, y: float, status: str) -> None:
        if not bool(self.get_parameter('seq.logging_enabled').value):
            return
        try:
            if self._seq_log_file is None:
                import csv
                self._seq_log_file = open(str(self.get_parameter('seq.log_path').value), 'a', newline='')
                self._seq_csv = csv.writer(self._seq_log_file)
                self._seq_csv.writerow(['stamp','cone_id','x','y','status'])
            self._seq_csv.writerow([f"{t:.6f}", cone_id, f"{x:.3f}", f"{y:.3f}", status])
            self._seq_log_file.flush()
        except Exception:
            pass

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ───────────── Persistent cones map (no TTL) ─────────────
    def _on_reset_persist(self, _msg) -> None:
        self._persist_cones.clear()
        self._persist_frozen = False
        self.get_logger().info('🧹 Persistent cone map reset.')

    def _persist_update_from_obs(self, msg: ObstacleArray) -> None:
        if not bool(self.get_parameter('persist.enable').value):
            return
        if self._persist_frozen and bool(self.get_parameter('persist.freeze_after_confirm').value):
            return
        merge_r = float(self.get_parameter('persist.merge_radius').value)
        mr2 = merge_r * merge_r
        for obs in msg.obstacles:
            # 차량 기준 오른쪽 콘만 유지(상위 필터와 일치) - 임시 비활성화
            # try:
            #     if hasattr(obs, 'description') and obs.description:
            #         if str(obs.description).strip().lower() != 'right':
            #             continue
            # except Exception:
            #     pass
            px = float(obs.center.x); py = float(obs.center.y)
            # 반경 내 가장 가까운 저장 콘과 병합
            best_i = -1; best_d2 = 1e9
            for i, c in enumerate(self._persist_cones):
                dx = c['x'] - px; dy = c['y'] - py
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; best_i = i
            if best_d2 <= mr2 and best_i >= 0:
                c = self._persist_cones[best_i]
                c['x'] = 0.7 * c['x'] + 0.3 * px
                c['y'] = 0.7 * c['y'] + 0.3 * py
            else:
                self._persist_cones.append({'x': px, 'y': py})
        
        # 디버그: 영속 맵 상태 로그
        if len(self._persist_cones) > 0:
            self.get_logger().info(f"영속 맵 콘 개수: {len(self._persist_cones)}")

    def _persist_get_cones(self) -> List[Tuple[float, float]]:
        if not bool(self.get_parameter('persist.enable').value):
            return []
        return [(c['x'], c['y']) for c in self._persist_cones]

    def _seq_update_cache(self, msg: ObstacleArray) -> None:
        now_s = self._now_sec()
        for obs in msg.obstacles:
            cid = int(obs.id)
            x = float(obs.center.x)
            y = float(obs.center.y)
            self._seq_cone_cache[cid] = {'x': x, 'y': y, 't': now_s}
            self._seq_log(now_s, cid, x, y, 'seen')
        # prune old
        ttl = float(self.get_parameter('seq.map_cache_ttl_sec').value)
        for cid in list(self._seq_cone_cache.keys()):
            if now_s - self._seq_cone_cache[cid]['t'] > ttl:
                st = self._seq_cone_cache.pop(cid)
                self._seq_log(now_s, cid, st['x'], st['y'], 'expired')

    def _seq_get_buffered_cones(self) -> List[Tuple[float, float]]:
        now_s = self._now_sec()
        win = float(self.get_parameter('seq.buffer_window_sec').value)
        cones = []
        for st in self._seq_cone_cache.values():
            if now_s - st['t'] <= win:
                cones.append((st['x'], st['y']))
        return cones

    def process_cones_sequential(self, cone_obstacles: ObstacleArray):
        # 노이즈 필터링 적용
        if bool(self.get_parameter('filter.enable_noise_removal').value):
            cone_obstacles = self._filter_noise_cones(cone_obstacles)
        
        # 통계적 안정성 업데이트
        self._update_cone_statistics(cone_obstacles)
        
        # 🎯 0순위: 통계적 안정성 기반 감지 (새로운 접근 방식)
        if self.statistical_enable:
            statistical_result = self._detect_parking_area_statistical()
            if statistical_result is not None:
                slot_id, center_pos = statistical_result
                self.get_logger().info(f"📊 통계적 안정성 감지 성공: 슬롯 {slot_id}, 위치 ({center_pos[0]:.2f}, {center_pos[1]:.2f})")
                self._publish_statistical_result(slot_id, center_pos)
                return
        
        # 🎯 1순위: 주차 패턴 분류 (가장 가까운 콘 기준)
        parking_pattern = self._classify_parking_pattern_by_closest_cone(cone_obstacles)
        
        # 🎯 2순위: Gap 기반 감지 (사용자 요구사항에 따른 방법 선택)
        gap_method = self.get_parameter('gap.method').get_parameter_value().string_value
        
        if gap_method == 'closest_cone':
            # 방법 1: 차량 가까운 콘 기준 갭 감지
            gap_result = self._detect_parking_gap_simple(cone_obstacles)
            method_name = f"가까운콘기준({parking_pattern})"
        elif gap_method == 'distance_range':
            # 방법 2: 4m~6m 거리 콘 쌍 기준
            gap_result = self._detect_parking_gap_by_distance_range(cone_obstacles)
            method_name = f"거리범위기준({parking_pattern})"
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 갭 감지 방법: {gap_method}")
            gap_result = self._detect_parking_gap_simple(cone_obstacles)
            method_name = f"기본방법({parking_pattern})"
        
        if gap_result is not None:
            gap_center, gap_id = gap_result
            self.get_logger().info(f"🎯 Gap 감지 성공 ({method_name}): 위치 ({gap_center[0]:.2f}, {gap_center[1]:.2f}), 슬롯 {gap_id}")
            self._publish_gap_based_result(gap_center, gap_id)
            return
        
        # 🎯 3순위: 패턴 매칭 시도 (정확한 콘 배치가 감지된 경우)
        if self.get_parameter('pattern.enable').get_parameter_value().bool_value:
            pattern_result = self._try_pattern_matching(cone_obstacles)
            if pattern_result is not None:
                pattern_name, slot_id, center_pos = pattern_result
                self.get_logger().info(f"🎯 패턴 매칭 성공: {pattern_name}, 슬롯 {slot_id}, 위치 ({center_pos[0]:.2f}, {center_pos[1]:.2f})")
                self._publish_pattern_result(center_pos, slot_id, pattern_name)
                return
        
        # 🎯 4순위: 최소 정보 기반 단순 주차 (최후의 수단)
        simple_result = self._find_parking_minimal(cone_obstacles)
        if simple_result is not None:
            parking_pose, method = simple_result
            self.get_logger().info(f"🎯 단순 주차 (fallback): 위치 ({parking_pose[0]:.2f}, {parking_pose[1]:.2f}), 방법: {method}")
            self._publish_simple_result(parking_pose, method)
            return
        
        self.get_logger().warn("❌ 모든 구역 감지 방법 실패 - 기존 sequential 로직으로 fallback")
        
        # 기존 sequential 로직 (fallback)
        self._seq_update_cache(cone_obstacles)
        self._persist_update_from_obs(cone_obstacles)
        buffered_cones = self._seq_get_buffered_cones()
        now_s = self._now_sec()
        buf_count = len(buffered_cones)

        # If buffer size dropped (frame drop), optionally skip updates to avoid oscillation
        if bool(self.get_parameter('seq.skip_on_drop').value) and buf_count < self._seq_prev_buffer_count:
            self._seq_prev_buffer_count = buf_count
            return None

        # 우선: 도로측 일렬 콘 기반 오픈 슬롯 판정 시도 (부분 관측 대응)
        # 의사결정에 사용할 콘 집합 선택(영속 우선)
        cones_for_decision = buffered_cones
        if bool(self.get_parameter('persist.use_for_decision').value):
            pcones = self._persist_get_cones()
            if pcones:
                cones_for_decision = pcones

        try:
            line_open_id = self.find_open_slot_line(cones_for_decision)
        except Exception:
            line_open_id = None

        if line_open_id is not None:
            # Geometry 재계산 및 안정 토픽 퍼블리시
            try:
                line_cones = self._extract_roadside_line_cones(cones_for_decision)
                self._publish_line_cones_debug(line_cones)  # 시각화 추가
            except Exception:
                line_cones = []
            if bool(self.get_parameter('align.use_line_only').value) and line_cones:
                self.adjust_parameters_to_line(line_cones)
            else:
                self.adjust_parameters_to_actual_cones(cones_for_decision)
            areas = self.build_parking_areas(cones_for_decision)
            for i, area in enumerate(areas):
                area.is_open = (i == line_open_id)
            virtual_walls = []
            if line_open_id >= 0 and line_open_id < len(areas):
                virtual_walls.extend(self.make_D_shaped_walls(areas[line_open_id]))
            for i, area in enumerate(areas):
                if not area.is_open:
                    virtual_walls.extend(self.make_rectangular_walls(area))
            self.publish_results_stable(areas, line_open_id, virtual_walls)
            if bool(self.get_parameter('persist.freeze_after_confirm').value):
                self._persist_frozen = True
            self._seq_prev_buffer_count = buf_count
            return None

        # Compute instantaneous scores for each pattern using buffered cones
        inst_scores = {}
        for pname, pdata in self.LAYOUT_PATTERNS.items():
            inst_scores[pname] = self.calculate_pattern_match_score(buffered_cones, pdata)
        self._seq_last_inst_scores = inst_scores

        # Update EMA scores
        alpha = float(self.get_parameter('seq.ema_alpha').value)
        for pname, s in inst_scores.items():
            prev = self._seq_ema_scores.get(pname, 0.0)
            self._seq_ema_scores[pname] = alpha * float(s) + (1.0 - alpha) * float(prev)

        # Provisional decision: use best instantaneous score
        if not inst_scores:
            # Fallback to dynamic if no cones buffered
            return self.process_cones_dynamic_fallback(buffered_cones)

        best_p = max(inst_scores.items(), key=lambda kv: kv[1])[0]
        min_score_delta = float(self.get_parameter('seq.min_score_delta').value)
        dwell_sec = float(self.get_parameter('seq.provisional_dwell_sec').value)

        # Update provisional candidate with stickiness
        if self._seq_prov_candidate is None:
            self._seq_prov_candidate = best_p
            self._seq_prov_since = now_s
        else:
            cur = self._seq_prov_candidate
            if best_p != cur:
                # Switch only if margin is significant
                if inst_scores[best_p] - inst_scores[cur] >= min_score_delta:
                    self._seq_prov_candidate = best_p
                    self._seq_prov_since = now_s

        # Apply pattern and build provisional areas only after dwell time
        if not bool(self.get_parameter('seq.publish_stable_only').value):
            if self._seq_prov_since is not None and (now_s - self._seq_prov_since) >= dwell_sec:
                pattern_data = self.LAYOUT_PATTERNS[self._seq_prov_candidate]
                self.open_area_id = pattern_data["open_slot"]
                self.detected_pattern = self._seq_prov_candidate
                self.adjust_parameters_to_actual_cones(buffered_cones)
                self.areas = self.build_parking_areas(buffered_cones)
                for i, area in enumerate(self.areas):
                    area.is_open = (i == self.open_area_id)
                self.virtual_walls = []
                if self.open_area_id >= 0 and self.open_area_id < len(self.areas):
                    self.virtual_walls.extend(self.make_D_shaped_walls(self.areas[self.open_area_id]))
                for i, area in enumerate(self.areas):
                    if not area.is_open:
                        self.virtual_walls.extend(self.make_rectangular_walls(area))
                self.publish_results()

        # Stable decision via EMA + hysteresis + hold time
        ema_best_p = max(self._seq_ema_scores.items(), key=lambda kv: kv[1])[0]
        ema_best = self._seq_ema_scores[ema_best_p]
        confirm_th = float(self.get_parameter('seq.confirm_threshold').value)
        release_th = float(self.get_parameter('seq.release_threshold').value)
        hold_sec = float(self.get_parameter('seq.stable_hold_sec').value)
        release_grace = float(self.get_parameter('seq.release_grace_sec').value)

        # Start/maintain candidate
        if ema_best >= confirm_th:
            if self._seq_stable_candidate != ema_best_p:
                self._seq_stable_candidate = ema_best_p
                self._seq_stable_since = now_s
            # Hold satisfied -> confirm stable pattern
            if self._seq_stable_since is not None and (now_s - self._seq_stable_since) >= hold_sec:
                self._seq_stable_pattern = ema_best_p
            # Reset release grace timer
            self._seq_release_below_since = None
        else:
            # Below confirm; if below release threshold, start grace timer to drop stable
            if self._seq_stable_pattern is not None:
                cur_stable_ema = self._seq_ema_scores.get(self._seq_stable_pattern, 0.0)
                if cur_stable_ema < release_th:
                    if self._seq_release_below_since is None:
                        self._seq_release_below_since = now_s
                    elif (now_s - self._seq_release_below_since) >= release_grace:
                        self._seq_stable_pattern = None
                        self._seq_stable_candidate = None
                        self._seq_stable_since = None
                        self._seq_release_below_since = None
                else:
                    # Recovered above release threshold
                    self._seq_release_below_since = None

        # Publish stable topics if available
        if self._seq_stable_pattern is not None:
            pstable = self.LAYOUT_PATTERNS[self._seq_stable_pattern]
            open_id = pstable['open_slot']
            # Recompute geometry using buffered cones for stability
            self.adjust_parameters_to_actual_cones(buffered_cones)
            areas = self.build_parking_areas(buffered_cones)
            for i, area in enumerate(areas):
                area.is_open = (i == open_id)
            virtual_walls = []
            if open_id >= 0 and open_id < len(areas):
                virtual_walls.extend(self.make_D_shaped_walls(areas[open_id]))
            for i, area in enumerate(areas):
                if not area.is_open:
                    virtual_walls.extend(self.make_rectangular_walls(area))
            self.publish_results_stable(areas, open_id, virtual_walls)
        self._seq_prev_buffer_count = buf_count
        return None

    def publish_results_stable(self, areas: List[ParkingArea], open_area_id: int, virtual_walls: List[VirtualWall]) -> None:
        # Stable virtual walls
        virtual_walls_msg = ObstacleArray()
        virtual_walls_msg.header.frame_id = "map"
        virtual_walls_msg.header.stamp = self.get_clock().now().to_msg()
        for i, wall in enumerate(virtual_walls):
            obstacle = Obstacle()
            obstacle.id = i + 2000
            obstacle.type = "virtual_wall_segment"
            cx = (wall.start_point[0] + wall.end_point[0]) / 2
            cy = (wall.start_point[1] + wall.end_point[1]) / 2
            obstacle.center.x = cx
            obstacle.center.y = cy
            obstacle.center.z = 0.0
            obstacle.radius = 0.0
            obstacle.description = f"seg:{wall.start_point[0]:.3f},{wall.start_point[1]:.3f},{wall.end_point[0]:.3f},{wall.end_point[1]:.3f},{wall.width:.3f}"
            virtual_walls_msg.obstacles.append(obstacle)
        self.virtual_walls_stable_pub.publish(virtual_walls_msg)

        # Stable open slot pose
        if areas and open_area_id < len(areas):
            open_slot_pose = PoseStamped()
            open_slot_pose.header.frame_id = "map"
            open_slot_pose.header.stamp = self.get_clock().now().to_msg()
            open_area = areas[open_area_id]
            open_slot_pose.pose.position.x = open_area.center[0]
            open_slot_pose.pose.position.y = open_area.center[1]
            open_slot_pose.pose.position.z = 0.0
            orientation = self._calculate_slot_orientation(open_area)
            open_slot_pose.pose.orientation = orientation
            self.open_slot_pose_stable_pub.publish(open_slot_pose)
        # Update internal state so that timer-driven RViz visualization reflects stable results
        self.areas = areas
        self.open_area_id = open_area_id
        self.virtual_walls = virtual_walls
        # Optionally trigger an immediate visualization publish
        try:
            self.publish_visualization()
        except Exception:
            pass
    
    def process_cones_dynamic_fallback(self, cone_positions: List[Tuple[float, float]]):
        """패턴 매칭 실패 시 기존 동적 로직 사용 (fallback)"""
        self.get_logger().warn('⚠️ Fallback: 기존 동적 분석 방식 사용')
        
        # 기존 로직 실행
        self.open_area_id = self.find_open_slot(cone_positions)
        self.areas = self.build_parking_areas(cone_positions)
        
        # 구역 열림/닫힘 상태 설정
        for i, area in enumerate(self.areas):
            area.is_open = (i == self.open_area_id)
        
        # 가상 벽 생성
        self.virtual_walls = []
        
        # 열린 구역: ㄷ-자 벽
        if self.open_area_id >= 0 and self.open_area_id < len(self.areas):
            open_walls = self.make_D_shaped_walls(self.areas[self.open_area_id])
            self.virtual_walls.extend(open_walls)
        
        # 닫힌 구역들: 완전한 사각형 벽
        for i, area in enumerate(self.areas):
            if not area.is_open:
                closed_walls = self.make_rectangular_walls(area)
                self.virtual_walls.extend(closed_walls)
        
        self.publish_results()
    
    def _calculate_slot_orientation(self, area: ParkingArea) -> Quaternion:
        """영역 경계선의 긴 변 방향에서 orientation 계산"""
        if len(area.boundary_points) < 4:
            # 기본 방향 (도로 진행 방향)
            q = Quaternion()
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            q.w = 1.0
            return q
        
        # 경계선들의 길이 계산
        edges = []
        for i in range(len(area.boundary_points)):
            start = area.boundary_points[i]
            end = area.boundary_points[(i + 1) % len(area.boundary_points)]
            
            length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            angle = np.arctan2(end[1] - start[1], end[0] - start[0])
            edges.append((length, angle))
        
        # 가장 긴 변의 방향을 사용
        longest_edge = max(edges, key=lambda x: x[0])
        yaw = longest_edge[1]
        
        # 쿼터니언으로 변환
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(yaw / 2.0)
        q.w = np.cos(yaw / 2.0)
        
        return q
    
    def broadcast_slot_transforms(self):
        """슬롯 프레임들을 TF로 브로드캐스트"""
        transforms = []
        
        for area in self.areas:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = f"slot_{area.id}"
            
            # 위치 설정
            transform.transform.translation.x = area.center[0]
            transform.transform.translation.y = area.center[1]
            transform.transform.translation.z = 0.0
            
            # 방향 설정
            orientation = self._calculate_slot_orientation(area)
            transform.transform.rotation = orientation
            
            transforms.append(transform)
        
        # 모든 변환을 한 번에 브로드캐스트
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
    

    
    # ──── ❸ 하드코딩된 슬롯 y-경계 ───────────────────────────────────────────────
    def slot_bounds(self, sid: int) -> Tuple[float, float]:
        """슬롯의 y 경계 계산"""
        # sid : 0=bottom, 1=mid, 2=top
        y_bottom = self.slot_origin_y + sid * (self.SLOT_LEN + self.SLOT_GAP) - self.EPS_Y
        y_top = y_bottom + self.SLOT_LEN + 2 * self.EPS_Y
        return (y_bottom, y_top)
    
    # ──── ❹ 좌측 콘 개수 세기 ───────────────────────────────────────────────
    def left_cone_count(self, cones: List[Tuple[float, float]], sid: int) -> int:
        """좌측 콘 개수 세기"""
        y_bottom, y_top = self.slot_bounds(sid)
        count = 0
        
        for x, y in cones:
            if (x < self.slot_origin_x + self.EPS_X and 
                y_bottom <= y <= y_top):
                count += 1
        
        return count
    
    # ──── ❹-b 도로측 일렬 콘 추출 및 카운트 ─────────────────────────────────────────
    def _extract_roadside_line_cones(self, cones: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """도로에 접한 일렬(열) 콘만 추출
        - 좌측 q-분위의 x 분포를 사용해 대표 x0를 잡고, |x-x0| <= tol 인 콘만 반환
        - 반환 리스트는 진행 방향(y 오름차순)으로 정렬
        - 포인트 수가 최소치 미만이면 빈 리스트 반환
        """
        if not cones:
            return []
        xs = sorted([x for x, _ in cones])
        q = float(self.get_parameter('line.quantile').value)
        tol = float(self.get_parameter('line.x_tolerance').value)
        min_pts = int(self.get_parameter('line.min_points').value)
        qn = max(1, int(len(xs) * max(0.05, min(0.9, q))))
        left_block = xs[:qn]
        # 대표 x0: 좌측 블록 중앙값
        if not left_block:
            return []
        mid = len(left_block) // 2
        x0 = left_block[mid] if len(left_block) % 2 == 1 else 0.5 * (left_block[mid - 1] + left_block[mid])
        # 허용 오차 내 일렬 필터
        line_cones = [(x, y) for x, y in cones if abs(x - x0) <= tol]
        # 충분히 세워진 열만 인정
        if len(line_cones) < min_pts:
            return []
        # 진행 방향 정렬(y 오름차순)
        line_cones.sort(key=lambda p: p[1])
        return line_cones

    def _publish_line_cones_debug(self, line_cones: List[Tuple[float, float]]) -> None:
        """도로측 라인 콘들을 MarkerArray로 시각화"""
        if not bool(self.get_parameter('debug.publish_line_cones').value):
            return
        
        marker_array = MarkerArray()
        
        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "line_cones"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 라인 콘들을 빨간 구체로 시각화
        for i, (x, y) in enumerate(line_cones):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "line_cones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.a = 1.0
            marker.color.r = 1.0  # 빨간색
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.line_cones_debug_pub.publish(marker_array)

    def _count_in_y_band(self, cones_line: List[Tuple[float, float]], y_bottom: float, y_top: float) -> int:
        """일렬 콘 집합에서 y-구간에 포함되는 포인트 수"""
        if not cones_line:
            return 0
        cnt = 0
        for _, y in cones_line:
            if y_bottom <= y <= y_top:
                cnt += 1
        return cnt

    def left_cone_count_line(self, cones_line: List[Tuple[float, float]], sid: int) -> int:
        """일렬 콘 집합을 사용해 슬롯 y-대역 내 개수 계산"""
        y_bottom, y_top = self.slot_bounds(sid)
        return self._count_in_y_band(cones_line, y_bottom, y_top)
    
    # ──── ❺ 열린 슬롯 탐색 (핵심 로직) ───────────────────────────────────────────────
    def find_open_slot(self, cones: List[Tuple[float, float]]) -> int:
        """열린 슬롯 탐색"""
        k = int(self.get_parameter('slot.cones_per_closed_band').value)
        for sid in [0, 1, 2]:  # 아래→위 순서
            if self.left_cone_count(cones, sid) < k:  # 0..k-1개 → open
                return sid
        return -1  # 예외: 모든 슬롯이 콘 3개 → 전부 닫힘
    
    def find_open_slot_line(self, cones: List[Tuple[float, float]]):
        """도로측 일렬 콘만으로 열린 슬롯 탐색 (실패 시 None)
        - 규칙(A): (축정렬) 각 슬롯 y-대역에서 일렬 콘 개수 < K 이면 open (K 파라미터)
        - 규칙(B): (PCA 정렬) s-축으로 3등분한 밴드에서 개수 < K 이면 open
        - 일렬 추출 실패 시 None 반환 (상위 로직이 폴백 처리)
        """
        use_pca = bool(self.get_parameter('line.use_pca').value)
        tol = float(self.get_parameter('line.distance_tolerance').value)
        k = int(self.get_parameter('slot.cones_per_closed_band').value)
        line_cones = self._extract_roadside_line_cones(cones)
        self._publish_line_cones_debug(line_cones)  # 시각화 추가
        if not line_cones:
            return None

        if not use_pca:
            for sid in [0, 1, 2]:
                if self.left_cone_count_line(line_cones, sid) < k:
                    return sid
            return -1

        # PCA 기반: 라인 방향(s)과 법선(n) 축으로 회전
        # 1) 평균 제거
        xs = np.array([p[0] for p in line_cones], dtype=float)
        ys = np.array([p[1] for p in line_cones], dtype=float)
        cx, cy = xs.mean(), ys.mean()
        X = np.stack([xs - cx, ys - cy], axis=0)  # 2 x N
        # 2) 공분산 및 고유벡터(주성분)
        C = np.cov(X)
        eigvals, eigvecs = np.linalg.eig(C)  # 2x2
        i_max = int(np.argmax(eigvals))
        v = eigvecs[:, i_max]   # 진행방향(s)
        n = np.array([-v[1], v[0]])  # 법선
        # 3) 각 점을 s,n 좌표로 투영
        s = v[0] * (xs - cx) + v[1] * (ys - cy)
        d = n[0] * (xs - cx) + n[1] * (ys - cy)
        # 4) 라인에서 tol 초과하는 이상치는 제외
        mask = np.abs(d) <= tol
        if not np.any(mask):
            return -1
        s_in = s[mask]
        # 5) (선택) 갭 기반 열린 구역 판정: 큰 간격이 있는 밴드를 open
        if bool(self.get_parameter('gap.use').value):
            s_sorted = np.sort(s_in)
            diffs = np.diff(s_sorted)
            if diffs.size > 0:
                min_gap = float(self.get_parameter('gap.min_length').value)
                # 후보 갭 인덱스
                cand_idx = np.where(diffs >= min_gap)[0]
                if cand_idx.size > 0:
                    strategy = str(self.get_parameter('gap.pick_strategy').value)
                    if strategy == 'first':
                        gi = int(cand_idx[0])
                    else:
                        gi = int(cand_idx[np.argmax(diffs[cand_idx])])
                    # 갭 중앙 s
                    s_mid = float(0.5 * (s_sorted[gi] + s_sorted[gi+1]))
                    # 3등분된 밴드로 매핑하여 해당 슬롯을 open으로 반환
                    s_min, s_max = float(np.min(s_in)), float(np.max(s_in))
                    span = max(1e-6, s_max - s_min)
                    band = span / 3.0
                    sid = int(np.clip(np.floor((s_mid - s_min) / band), 0, 2))
                    return sid

        # 6) s-축을 3등분한 밴드로 개수 판정
        s_min, s_max = float(np.min(s_in)), float(np.max(s_in))
        span = max(1e-6, s_max - s_min)
        band = span / 3.0
        def count_in_band(idx):
            b0 = s_min + idx * band
            b1 = b0 + band
            return int(np.sum((s_in >= b0) & (s_in <= b1)))
        for sid in [0, 1, 2]:
            if count_in_band(sid) < k:
                return sid
        return -1
    
    # ──── ❻ 구역 객체 채우기 ───────────────────────────────────────────────
    def build_parking_areas(self, cones: List[Tuple[float, float]]) -> List[ParkingArea]:
        """구역 객체 채우기 - 라인 콘 방향에 맞춰 회전된 구역 생성"""
        areas = []
        
        # 라인 콘 방향 계산 (라인 기반 정렬이 활성화된 경우)
        if bool(self.get_parameter('align.use_line_only').value):
            try:
                # 일단 모든 콘으로 전체 방향 계산 (임시)
                line_cones = self._extract_roadside_line_cones(cones)
                if len(line_cones) < 2 and len(cones) >= 2:
                    # 라인 추출 실패 시 모든 콘으로 시도
                    line_cones = cones
                    self.get_logger().info("⚠️ 라인 추출 실패, 모든 콘으로 방향 계산")
                
                if len(line_cones) >= 2:
                    # 더 간단하고 직관적인 라인 방향 계산
                    xs = [p[0] for p in line_cones]
                    ys = [p[1] for p in line_cones]
                    
                    # 최소제곱법으로 직선 기울기 계산
                    n = len(line_cones)
                    sum_x = sum(xs)
                    sum_y = sum(ys)
                    sum_xy = sum(x*y for x, y in zip(xs, ys))
                    sum_x2 = sum(x*x for x in xs)
                    
                    # 분모가 0에 가까우면 수직선 (기울기 무한대)
                    denominator = n * sum_x2 - sum_x * sum_x
                    if abs(denominator) < 1e-6:
                        line_angle = math.pi / 2  # 90도 (수직선)
                    else:
                        slope = (n * sum_xy - sum_x * sum_y) / denominator
                        line_angle = math.atan(slope)
                    
                    # 디버깅 정보 출력
                    self.get_logger().info(f"🔄 라인 방향: {math.degrees(line_angle):.1f}도")
                    self.get_logger().info(f"   라인 콘 개수: {len(line_cones)}")
                    self.get_logger().info(f"   콘 좌표: {line_cones[:5]}...")  # 처음 5개만
                    
                    # 각도가 유의미한 경우에만 회전 적용 (5도 이상)
                    if abs(math.degrees(line_angle)) > 5.0:
                        return self._build_rotated_areas(cones, line_angle)
                    else:
                        self.get_logger().info("   각도가 작아 기본 구역 사용")
            except Exception as e:
                self.get_logger().warn(f"라인 방향 계산 실패: {e}")
        
        # 기본 방식: X/Y 축 평행 구역
        for sid in [0, 1, 2]:
            y_bottom, y_top = self.slot_bounds(sid)
            
            # 경계점 생성 (사각형)
            boundary = [
                (self.slot_origin_x, y_bottom),                    # 좌하
                (self.slot_origin_x + self.SLOT_WIDTH, y_bottom),  # 우하
                (self.slot_origin_x + self.SLOT_WIDTH, y_top),     # 우상
                (self.slot_origin_x, y_top)                        # 좌상
            ]
            
            # 해당 슬롯에 있는 콘들 필터링
            cones_in_slot = [(x, y) for x, y in cones
                            if (self.slot_origin_x <= x <= self.slot_origin_x + self.SLOT_WIDTH and
                                y_bottom <= y <= y_top)]
            
            area = ParkingArea(sid, boundary, cones_in_slot)
            areas.append(area)
        
        return areas
    
    def _filter_noise_cones(self, cone_obstacles: ObstacleArray) -> ObstacleArray:
        """노이즈 콘 제거: 거리 필터링 + 클러스터링 기반 아웃라이어 제거"""
        if not cone_obstacles.obstacles:
            return cone_obstacles
        
        max_dist = float(self.get_parameter('filter.max_distance_from_vehicle').value)
        min_cluster_size = int(self.get_parameter('filter.min_cluster_size').value)
        cluster_radius = float(self.get_parameter('filter.cluster_radius').value)
        
        # 1. 거리 필터링 (차량으로부터 너무 먼 콘 제거)
        filtered_obstacles = []
        for obs in cone_obstacles.obstacles:
            dist = math.sqrt(obs.center.x**2 + obs.center.y**2)
            if dist <= max_dist:
                filtered_obstacles.append(obs)
        
        if len(filtered_obstacles) < 2:
            cone_obstacles.obstacles = filtered_obstacles
            return cone_obstacles
        
        # 2. 클러스터링 기반 아웃라이어 제거
        positions = [(obs.center.x, obs.center.y) for obs in filtered_obstacles]
        clusters = self._simple_clustering(positions, cluster_radius)
        
        # 3. 큰 클러스터의 콘들만 유지
        valid_indices = set()
        for cluster in clusters:
            if len(cluster) >= min_cluster_size:
                valid_indices.update(cluster)
        
        final_obstacles = [filtered_obstacles[i] for i in sorted(valid_indices)]
        
        if len(final_obstacles) != len(cone_obstacles.obstacles):
            self.get_logger().info(f"🧹 노이즈 필터링: {len(cone_obstacles.obstacles)} → {len(final_obstacles)} 콘")
        
        cone_obstacles.obstacles = final_obstacles
        return cone_obstacles
    
    def _simple_clustering(self, positions: List[Tuple[float, float]], radius: float) -> List[List[int]]:
        """간단한 거리 기반 클러스터링"""
        n = len(positions)
        visited = [False] * n
        clusters = []
        
        for i in range(n):
            if visited[i]:
                continue
            
            # 새로운 클러스터 시작
            cluster = []
            stack = [i]
            
            while stack:
                curr = stack.pop()
                if visited[curr]:
                    continue
                
                visited[curr] = True
                cluster.append(curr)
                
                # 반경 내 이웃들 찾기
                for j in range(n):
                    if not visited[j]:
                        dist = math.sqrt((positions[curr][0] - positions[j][0])**2 + 
                                       (positions[curr][1] - positions[j][1])**2)
                        if dist <= radius:
                            stack.append(j)
            
            if cluster:
                clusters.append(cluster)
        
        return clusters
    
    def _find_parking_minimal(self, cone_obstacles: ObstacleArray) -> Optional[Tuple[Tuple[float, float, float], str]]:
        """현실적 접근: 최소 정보로 주차 위치 결정"""
        
        # 1. 사전 정의된 주차 구역 (대회장 고정 위치)
        predefined_zones = [
            (3.0, -3.0, 0.0),    # 구역 0: 가까운 곳
            (7.0, -3.0, 0.0),    # 구역 1: 중간
            (11.0, -3.0, 0.0)    # 구역 2: 먼 곳
        ]
        
        # 2. 콘 개수별 전략
        cone_count = len(cone_obstacles.obstacles)
        
        if cone_count == 0:
            # 콘 없음: 기본 위치 사용
            return (predefined_zones[0], "기본위치")
        
        elif cone_count == 1:
            # 콘 1개: 그 콘 기준 상대 위치
            cone = cone_obstacles.obstacles[0]
            x = cone.center.x + 2.0  # 콘에서 2m 앞
            y = cone.center.y - 1.5  # 콘에서 1.5m 우측
            return ((x, y, 0.0), "1콘기준")
        
        elif cone_count == 2:
            # 콘 2개: 중점 기준
            cone1 = cone_obstacles.obstacles[0]
            cone2 = cone_obstacles.obstacles[1]
            x = (cone1.center.x + cone2.center.x) / 2 + 1.0
            y = (cone1.center.y + cone2.center.y) / 2 - 0.5
            return ((x, y, 0.0), "2콘중점")
        
        else:
            # 콘 3개 이상: 가장 가까운 2개만 사용
            distances = [(i, cone.center.x**2 + cone.center.y**2) 
                        for i, cone in enumerate(cone_obstacles.obstacles)]
            distances.sort(key=lambda x: x[1])
            
            closest_indices = [distances[0][0], distances[1][0]]
            cone1 = cone_obstacles.obstacles[closest_indices[0]]
            cone2 = cone_obstacles.obstacles[closest_indices[1]]
            
            x = (cone1.center.x + cone2.center.x) / 2 + 1.0
            y = (cone1.center.y + cone2.center.y) / 2 - 0.5
            return ((x, y, 0.0), "근접2콘")
    
    def _publish_simple_result(self, parking_pose: Tuple[float, float, float], method: str):
        """단순 결과 퍼블리시"""
        x, y, theta = parking_pose
        
        # 1. 슬롯 ID 계산 (x 위치 기반)
        if x < 5.0:
            slot_id = 0
        elif x < 10.0:
            slot_id = 1
        else:
            slot_id = 2
        
        # 2. 열린 구역 ID 퍼블리시
        area_msg = Int32()
        area_msg.data = slot_id
        self.open_area_pub.publish(area_msg)
        
        # 3. 정규좌표계를 map 좌표계로 변환
        map_x, map_y = self._transform_normalized_to_map(x, y)
        
        # 4. 주차 포즈 생성 (변환된 좌표 기준)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 0도 방향
        
        # 5. 결과 퍼블리시
        self.open_slot_pose_pub.publish(pose)
        self.open_slot_pose_stable_pub.publish(pose)
        
        self.get_logger().info(f"✅ 단순 주차 포즈: map 좌표 ({map_x:.2f}, {map_y:.2f}) by {method}")
        self.get_logger().info(f"   (정규좌표: ({x:.2f}, {y:.2f}) → map: ({map_x:.2f}, {map_y:.2f}))")
        
        # 6. 시각화를 위한 간단한 구역 생성 및 퍼블리시
        self._create_and_publish_simple_visualization(map_x, map_y, slot_id)
    
    def _find_parking_spot_direct(self, cone_obstacles: ObstacleArray) -> Optional[Tuple[Tuple[float, float, float], float]]:
        """Occupancy Grid 기반 직접 주차 스팟 탐색"""
        if not cone_obstacles.obstacles:
            return None
        
        # 1. 차량 및 탐색 영역 파라미터
        car_length = 4.5  # 차량 길이 + 안전 여유
        car_width = 2.0   # 차량 폭 + 안전 여유
        search_range_x = (0, 12)  # x 탐색 범위
        search_range_y = (-6, 1)  # y 탐색 범위 (우측)
        grid_resolution = 0.3     # 격자 해상도
        
        # 2. 콘 위치 추출
        cone_positions = [(obs.center.x, obs.center.y) for obs in cone_obstacles.obstacles]
        
        # 3. 격자 기반 탐색
        best_pose = None
        best_score = 0.0
        
        x_range = np.arange(search_range_x[0], search_range_x[1], grid_resolution)
        y_range = np.arange(search_range_y[0], search_range_y[1], grid_resolution)
        
        for x in x_range:
            for y in y_range:
                for theta in [0, math.pi/6, -math.pi/6]:  # 0°, ±30°
                    score = self._evaluate_parking_pose(x, y, theta, cone_positions, car_length, car_width)
                    if score > best_score:
                        best_score = score
                        best_pose = (x, y, theta)
        
        # 4. 최소 신뢰도 체크
        if best_score > 0.5:  # 50% 이상 신뢰도
            return (best_pose, best_score)
        
        return None
    
    def _evaluate_parking_pose(self, x: float, y: float, theta: float, 
                              cones: List[Tuple[float, float]], 
                              car_length: float, car_width: float) -> float:
        """주차 포즈의 타당성 점수 계산 (0~1)"""
        
        # 차량 네 모서리 계산
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        half_l, half_w = car_length/2, car_width/2
        
        corners = [
            (x + cos_t*half_l - sin_t*half_w, y + sin_t*half_l + cos_t*half_w),  # 우상
            (x + cos_t*half_l + sin_t*half_w, y + sin_t*half_l - cos_t*half_w),  # 좌상  
            (x - cos_t*half_l + sin_t*half_w, y - sin_t*half_l - cos_t*half_w),  # 좌하
            (x - cos_t*half_l - sin_t*half_w, y - sin_t*half_l + cos_t*half_w),  # 우하
        ]
        
        # 1. 충돌 검사 (가장 중요)
        min_clearance = float('inf')
        for corner_x, corner_y in corners:
            for cone_x, cone_y in cones:
                dist = math.sqrt((corner_x - cone_x)**2 + (corner_y - cone_y)**2)
                min_clearance = min(min_clearance, dist)
        
        if min_clearance < 0.5:  # 0.5m 미만이면 충돌
            return 0.0
        
        # 2. 점수 계산
        clearance_score = min(1.0, min_clearance / 2.0)  # 2m 이상이면 만점
        
        # 3. 위치 선호도 (차량에서 가까운 곳 선호)
        distance_from_car = math.sqrt(x*x + y*y)
        distance_score = max(0, 1.0 - distance_from_car / 10.0)  # 10m 이상이면 0점
        
        # 4. 각도 선호도 (직진 선호)
        angle_score = max(0, 1.0 - abs(theta) / (math.pi/3))  # 60도 이상이면 0점
        
        # 가중 평균
        total_score = (clearance_score * 0.6 + distance_score * 0.3 + angle_score * 0.1)
        
        return total_score
    
    def _publish_direct_result(self, parking_pose: Tuple[float, float, float], confidence: float):
        """직접 탐색 결과 퍼블리시"""
        x, y, theta = parking_pose
        
        # 1. 슬롯 ID 계산 (x 위치 기반)
        if x < 4.0:
            slot_id = 0
        elif x < 8.0:
            slot_id = 1
        else:
            slot_id = 2
        
        # 2. 열린 구역 ID 퍼블리시
        self.open_area_id = slot_id
        area_msg = Int32()
        area_msg.data = slot_id
        self.open_area_pub.publish(area_msg)
        
        # 3. 주차 포즈 생성
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 쿼터니언 생성 (theta 회전)
        pose.pose.orientation.z = math.sin(theta / 2)
        pose.pose.orientation.w = math.cos(theta / 2)
        
        # 4. 결과 퍼블리시
        self.open_slot_pose_pub.publish(pose)
        self.open_slot_pose_stable_pub.publish(pose)
        
        self.get_logger().info(f"✅ 직접 탐색 주차 포즈: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)")
    
    def _detect_parking_gap_simple(self, cone_obstacles: ObstacleArray) -> Optional[Tuple[Tuple[float, float], int]]:
        """사용자 요구사항: 차량 근처 콘을 기준으로 다른 콘과 3m 초과 거리면 열린 갭으로 판단"""
        if not cone_obstacles.obstacles:
            return None
        
        # 1. 모든 콘 좌표 수집
        all_cones = []
        for obs in cone_obstacles.obstacles:
            distance_to_vehicle = math.sqrt(obs.center.x**2 + obs.center.y**2)
            all_cones.append((obs.center.x, obs.center.y, distance_to_vehicle))
        
        if len(all_cones) < 2:
            return None
        
        # 2. 차량과 가장 가까운 콘 찾기
        all_cones.sort(key=lambda p: p[2])  # 거리순 정렬
        closest_cone = all_cones[0]
        closest_x, closest_y, closest_dist = closest_cone
        
        self.get_logger().info(f"🎯 기준 콘: ({closest_x:.1f},{closest_y:.1f}) 차량거리={closest_dist:.1f}m")
        
        # 3. 기준 콘과 다른 모든 콘들 간의 거리 계산
        min_gap = float(self.get_parameter('gap.min_length').value)  # 3.0m
        gap_candidates = []
        
        for other_x, other_y, other_dist in all_cones[1:]:  # 기준 콘 제외
            # 기준 콘과 다른 콘 간의 거리
            cone_to_cone_distance = math.sqrt((other_x - closest_x)**2 + (other_y - closest_y)**2)
            
            if cone_to_cone_distance > min_gap:
                # 갭 중심점 계산
                gap_center = ((closest_x + other_x) / 2, (closest_y + other_y) / 2)
                gap_candidates.append((gap_center, cone_to_cone_distance, closest_cone, (other_x, other_y, other_dist)))
                
                self.get_logger().info(f"   ✅ 갭 후보: {cone_to_cone_distance:.2f}m ({closest_x:.1f},{closest_y:.1f}) ↔ ({other_x:.1f},{other_y:.1f})")
        
        if not gap_candidates:
            self.get_logger().info(f"   ❌ 충분한 갭 없음 (최소 {min_gap:.1f}m 필요)")
            return None
        
        # 4. 가장 가까운 갭 선택 (차량과 가까운 갭 우선)
        gap_candidates.sort(key=lambda g: math.sqrt(g[0][0]**2 + g[0][1]**2))  # 갭 중심점의 차량 거리순
        best_gap_center, best_distance, cone1, cone2 = gap_candidates[0]
        
        # 5. map 좌표계로 변환하여 슬롯 ID 계산
        map_x, map_y = self._transform_normalized_to_map(best_gap_center[0], best_gap_center[1])
        
        if map_x < 5.0:
            slot_id = 0  # 가까운 슬롯
        elif map_x < 10.0:
            slot_id = 1  # 중간 슬롯
        else:
            slot_id = 2  # 먼 슬롯
        
        self.get_logger().info(f"   🎯 최종 갭: {best_distance:.2f}m, 중심=({best_gap_center[0]:.1f},{best_gap_center[1]:.1f})")
        self.get_logger().info(f"   슬롯 계산: 정규({best_gap_center[0]:.1f},{best_gap_center[1]:.1f}) → map({map_x:.1f},{map_y:.1f}) → 슬롯{slot_id}")
        
        return (best_gap_center, slot_id)
    
    def _detect_parking_gap_by_distance_range(self, cone_obstacles: ObstacleArray) -> Optional[Tuple[Tuple[float, float], int]]:
        """방법 2: 콘 맵에서 4m~6m 거리의 콘 쌍을 찾아 선분 중심점을 주차 위치로 설정"""
        if not cone_obstacles.obstacles:
            return None
        
        # 1. 모든 콘 좌표 수집
        all_cones = []
        for obs in cone_obstacles.obstacles:
            all_cones.append((obs.center.x, obs.center.y))
        
        if len(all_cones) < 2:
            return None
        
        # 2. 파라미터에서 거리 범위 가져오기
        min_distance = float(self.get_parameter('gap.distance_range_min').value)  # 4.0m
        max_distance = float(self.get_parameter('gap.distance_range_max').value)  # 6.0m
        valid_pairs = []
        
        for i in range(len(all_cones)):
            for j in range(i + 1, len(all_cones)):
                x1, y1 = all_cones[i]
                x2, y2 = all_cones[j]
                
                # 두 콘 간의 거리 계산
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                if min_distance <= distance <= max_distance:
                    # 선분 중심점 계산
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # 차량으로부터 중심점까지의 거리
                    center_dist_to_vehicle = math.sqrt(center_x**2 + center_y**2)
                    
                    valid_pairs.append((
                        (center_x, center_y),  # 중심점
                        distance,              # 콘 간 거리
                        center_dist_to_vehicle,  # 차량-중심점 거리
                        (x1, y1), (x2, y2)    # 원본 콘 좌표
                    ))
                    
                    self.get_logger().info(f"   ✅ 유효 쌍: {distance:.2f}m ({x1:.1f},{y1:.1f}) ↔ ({x2:.1f},{y2:.1f}), 중심=({center_x:.1f},{center_y:.1f})")
        
        if not valid_pairs:
            self.get_logger().info(f"   ❌ {min_distance}m~{max_distance}m 범위의 콘 쌍 없음")
            return None
        
        # 3. 차량과 가장 가까운 중심점 선택
        valid_pairs.sort(key=lambda p: p[2])  # 차량-중심점 거리순 정렬
        best_center, best_cone_distance, best_vehicle_distance, cone1, cone2 = valid_pairs[0]
        
        # 4. map 좌표계로 변환하여 슬롯 ID 계산
        map_x, map_y = self._transform_normalized_to_map(best_center[0], best_center[1])
        
        if map_x < 5.0:
            slot_id = 0  # 가까운 슬롯
        elif map_x < 10.0:
            slot_id = 1  # 중간 슬롯
        else:
            slot_id = 2  # 먼 슬롯
        
        self.get_logger().info(f"   🎯 최종 선택: 콘간거리={best_cone_distance:.2f}m, 차량거리={best_vehicle_distance:.2f}m")
        self.get_logger().info(f"   슬롯 계산: 정규({best_center[0]:.1f},{best_center[1]:.1f}) → map({map_x:.1f},{map_y:.1f}) → 슬롯{slot_id}")
        
        return (best_center, slot_id)
    
    def _classify_parking_pattern_by_closest_cone(self, cone_obstacles: ObstacleArray) -> Optional[str]:
        """가장 가까운 콘을 기준으로 3가지 주차 배열 패턴 분류"""
        if not cone_obstacles.obstacles:
            return None
        
        # 1. 모든 콘의 차량으로부터 거리 계산
        cones_with_distance = []
        for obs in cone_obstacles.obstacles:
            distance = math.sqrt(obs.center.x**2 + obs.center.y**2)
            cones_with_distance.append((obs.center.x, obs.center.y, distance))
        
        if len(cones_with_distance) < 2:
            return None
        
        # 2. 거리순 정렬하여 가장 가까운 콘 찾기
        cones_with_distance.sort(key=lambda c: c[2])
        closest_cone = cones_with_distance[0]
        closest_x, closest_y, closest_dist = closest_cone
        
        self.get_logger().info(f"📍 가장 가까운 콘: ({closest_x:.1f}, {closest_y:.1f}) 거리={closest_dist:.1f}m")
        
        # 3. 가까운 콘 앞뒤로 다른 콘들 분포 확인
        cones_ahead = []    # 가까운 콘보다 앞쪽(x가 큰) 콘들
        cones_behind = []   # 가까운 콘보다 뒤쪽(x가 작은) 콘들
        
        for x, y, dist in cones_with_distance[1:]:  # 가장 가까운 콘 제외
            if x > closest_x + 1.0:  # 1m 이상 앞쪽
                cones_ahead.append((x, y, dist))
            elif x < closest_x - 1.0:  # 1m 이상 뒤쪽
                cones_behind.append((x, y, dist))
        
        # 4. 패턴 분류
        ahead_count = len(cones_ahead)
        behind_count = len(cones_behind)
        
        self.get_logger().info(f"   앞쪽 콘: {ahead_count}개, 뒤쪽 콘: {behind_count}개")
        
        if behind_count == 0 and ahead_count >= 2:
            pattern = "하단열림"  # 뒤에 콘이 없고 앞에만 있음
        elif behind_count >= 1 and ahead_count >= 1:
            pattern = "중간열림"  # 앞뒤로 콘이 모두 있음
        elif behind_count >= 2 and ahead_count == 0:
            pattern = "상단열림"  # 앞에 콘이 없고 뒤에만 있음
        else:
            pattern = "불명확"   # 판단하기 어려운 경우
        
        self.get_logger().info(f"🎯 패턴 분류: {pattern}")
        return pattern
    
    def _transform_normalized_to_map(self, normalized_x: float, normalized_y: float) -> Tuple[float, float]:
        """정규좌표계에서 map 좌표계로 역변환"""
        if not hasattr(self, '_cone_map_origin') or not hasattr(self, '_cone_map_rotation'):
            self.get_logger().warn("⚠️ 콘 맵 좌표계 정보가 없어 변환 불가")
            return (normalized_x, normalized_y)
        
        # 1. 정규좌표계에서 역회전 (각도의 음수 적용)
        cos_angle = math.cos(self._cone_map_rotation)  # 원래 각도 그대로
        sin_angle = math.sin(self._cone_map_rotation)
        
        # 역회전 적용
        rotated_x = normalized_x * cos_angle - normalized_y * sin_angle
        rotated_y = normalized_x * sin_angle + normalized_y * cos_angle
        
        # 2. 원점 오프셋 추가하여 map 좌표로 변환
        map_x = rotated_x + self._cone_map_origin[0]
        map_y = rotated_y + self._cone_map_origin[1]
        
        self.get_logger().debug(f"🔄 좌표 변환: ({normalized_x:.2f}, {normalized_y:.2f}) → ({map_x:.2f}, {map_y:.2f})")
        return (map_x, map_y)

    def _publish_gap_based_result(self, gap_center: Tuple[float, float], slot_id: int):
        """Gap 기반 간단한 결과 퍼블리시"""
        # 1. 열린 구역 ID 퍼블리시
        self.open_area_id = slot_id
        area_msg = Int32()
        area_msg.data = slot_id
        self.open_area_pub.publish(area_msg)
        
        # 2. 정규좌표계를 map 좌표계로 변환
        map_x, map_y = self._transform_normalized_to_map(gap_center[0], gap_center[1])
        
        # 3. 간단한 주차 포즈 계산 (변환된 좌표 기준)
        x_offset = 1.0  # 1m 앞으로
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x + x_offset
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 0도 방향
        
        # 4. 결과 퍼블리시
        self.open_slot_pose_pub.publish(pose)
        self.open_slot_pose_stable_pub.publish(pose)
        
        self.get_logger().info(f"✅ Gap 기반 주차 포즈: map 좌표 ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.get_logger().info(f"   (정규좌표: ({gap_center[0]:.2f}, {gap_center[1]:.2f}) → map: ({map_x:.2f}, {map_y:.2f}))")
        
        # 5. 시각화를 위한 간단한 구역 생성 및 퍼블리시
        self._create_and_publish_simple_visualization(pose.pose.position.x, pose.pose.position.y, slot_id)
    
    def _build_rotated_areas(self, cones: List[Tuple[float, float]], line_angle: float) -> List[ParkingArea]:
        """라인 방향에 맞춰 회전된 구역 생성"""
        areas = []
        cos_a, sin_a = math.cos(line_angle), math.sin(line_angle)
        
        self.get_logger().info(f"🏗️ 회전된 구역 생성: {math.degrees(line_angle):.1f}도")
        self.get_logger().info(f"   cos={cos_a:.3f}, sin={sin_a:.3f}")
        self.get_logger().info(f"   기준점: ({self.slot_origin_x:.2f}, {self.slot_origin_y:.2f})")
        
        for sid in [0, 1, 2]:
            y_bottom, y_top = self.slot_bounds(sid)
            
            # 로컬 좌표계에서 사각형 꼭짓점 정의 (slot_origin 기준)
            local_corners = [
                (0, y_bottom - self.slot_origin_y),                    # 좌하
                (self.SLOT_WIDTH, y_bottom - self.slot_origin_y),      # 우하
                (self.SLOT_WIDTH, y_top - self.slot_origin_y),         # 우상
                (0, y_top - self.slot_origin_y)                        # 좌상
            ]
            
            # 회전 변환 적용 후 글로벌 좌표로 변환
            boundary = []
            for lx, ly in local_corners:
                # 회전 변환 (시계방향)
                rx = lx * cos_a - ly * sin_a
                ry = lx * sin_a + ly * cos_a
                # 평행 이동
                gx = rx + self.slot_origin_x
                gy = ry + self.slot_origin_y
                boundary.append((gx, gy))
            
            # 디버깅: 첫 번째 구역의 경계점 출력
            if sid == 0:
                self.get_logger().info(f"   구역 {sid} 경계점: {[(f'{x:.2f}', f'{y:.2f}') for x, y in boundary]}")
            
            # 회전된 바운딩 박스 내 콘 필터링 (간단한 근사)
            cones_in_slot = []
            for x, y in cones:
                # 글로벌 좌표를 로컬 좌표로 역변환
                dx, dy = x - self.slot_origin_x, y - self.slot_origin_y
                lx = dx * cos_a + dy * sin_a
                ly = -dx * sin_a + dy * cos_a
                
                # 로컬 좌표계에서 범위 체크
                if (0 <= lx <= self.SLOT_WIDTH and 
                    y_bottom - self.slot_origin_y <= ly <= y_top - self.slot_origin_y):
                    cones_in_slot.append((x, y))
            
            area = ParkingArea(sid, boundary, cones_in_slot)
            areas.append(area)
        
        return areas
    
    # ──── ❼ ㄷ-자 가상 벽 생성 ───────────────────────────────────────────────
    def make_D_shaped_walls(self, area: ParkingArea) -> List[VirtualWall]:
        """ㄷ-자 가상 벽 생성 (열린 구역용)"""
        bp = area.boundary_points  # [좌하, 우하, 우상, 좌상]
        seg = [(3, 2), (2, 1), (1, 0)]
        walls = []
        
        for i, j in seg:
            start = bp[i]
            end = bp[j]
            # 가상 벽 폭을 얇게 고정 (충돌 모델 완화 목적)
            width = 0.01
            walls.append(VirtualWall(start, end, width))
        
        return walls
    
    # ──── ❼ 완전한 사각형 가상 벽 생성 ───────────────────────────────────────────────
    def make_rectangular_walls(self, area: ParkingArea) -> List[VirtualWall]:
        """완전한 사각형 가상 벽 생성 (닫힌 구역용)"""
        bp = area.boundary_points  # [좌하, 우하, 우상, 좌상]
        walls = []
        
        # 4면 모두 벽 생성
        for i in range(4):
            start = bp[i]
            end = bp[(i + 1) % 4]  # 다음 점 (마지막은 첫 번째로)
            # 가상 벽 폭을 얇게 고정 (충돌 모델 완화 목적)
            width = 0.01
            walls.append(VirtualWall(start, end, width))
        
        return walls
    
    def publish_results(self):
        """결과를 퍼블리시"""
        # 열린 구역 ID 퍼블리시
        open_area_msg = Int32()
        open_area_msg.data = self.open_area_id
        self.open_area_pub.publish(open_area_msg)
        
        # 가상 벽을 장애물로 변환하여 퍼블리시
        virtual_walls_msg = ObstacleArray()
        virtual_walls_msg.header.frame_id = "map"
        virtual_walls_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, wall in enumerate(self.virtual_walls):
            obstacle = Obstacle()
            obstacle.id = i + 1000  # 라바콘과 구분하기 위해 1000부터 시작
            obstacle.type = "virtual_wall_segment"
            
            # 벽의 중점을 중심으로 설정
            center_x = (wall.start_point[0] + wall.end_point[0]) / 2
            center_y = (wall.start_point[1] + wall.end_point[1]) / 2
            
            obstacle.center.x = center_x
            obstacle.center.y = center_y
            obstacle.center.z = 0.0
            
            # 원형 표현 사용 금지: 반지름 0으로 고정
            obstacle.radius = 0.0

            # 선형 장애물 정보 인코딩 (시작점, 끝점, 폭)
            obstacle.description = f"seg:{wall.start_point[0]:.3f},{wall.start_point[1]:.3f},{wall.end_point[0]:.3f},{wall.end_point[1]:.3f},{wall.width:.3f}"
            
            virtual_walls_msg.obstacles.append(obstacle)
        
        self.virtual_walls_pub.publish(virtual_walls_msg)
        
        # 열린 슬롯 중심점 퍼블리시
        if self.areas and self.open_area_id < len(self.areas):
            open_slot_pose = PoseStamped()
            open_slot_pose.header.frame_id = "map"
            open_slot_pose.header.stamp = self.get_clock().now().to_msg()
            
            open_area = self.areas[self.open_area_id]
            open_slot_pose.pose.position.x = open_area.center[0]
            open_slot_pose.pose.position.y = open_area.center[1]
            open_slot_pose.pose.position.z = 0.0
            
            # 영역 경계선의 긴 변 방향에서 orientation 계산
            orientation = self._calculate_slot_orientation(open_area)
            open_slot_pose.pose.orientation = orientation
            
            self.open_slot_pose_pub.publish(open_slot_pose)
        
        # TF 브로드캐스트 (슬롯 프레임들)
        self.broadcast_slot_transforms()
    
    def publish_visualization(self):
        """Rviz 시각화 마커 퍼블리시"""
        if not self.areas:
            return
        
        visualization_markers = self.create_visualization_markers()
        self.visualization_pub.publish(visualization_markers)
    
    def create_visualization_markers(self) -> MarkerArray:
        """모든 시각화 요소를 포함하는 마커 배열 생성"""
        markers = MarkerArray()
        
        # 구역 경계선
        boundary_markers = self.create_area_boundary_markers()
        markers.markers.extend(boundary_markers.markers)
        
        # 열린 구역 하이라이트
        if self.areas and self.open_area_id < len(self.areas):
            open_area_marker = self.create_open_area_marker(self.areas[self.open_area_id])
            markers.markers.append(open_area_marker)
        
        # 가상 벽
        wall_markers = self.create_virtual_wall_markers()
        markers.markers.extend(wall_markers.markers)
        
        # 구역 라벨
        label_markers = self.create_area_labels()
        markers.markers.extend(label_markers.markers)
        
        return markers
    
    def create_area_boundary_markers(self) -> MarkerArray:
        """3개 구역의 경계선을 시각화하는 마커 생성"""
        markers = MarkerArray()
        
        colors = [
            (1.0, 0.0, 0.0, 0.8),  # 빨간색 - 구역 0 (위)
            (0.0, 1.0, 0.0, 0.8),  # 초록색 - 구역 1 (중)
            (0.0, 0.0, 1.0, 0.8)   # 파란색 - 구역 2 (아래)
        ]
        
        for area_id, area in enumerate(self.areas):
            if area_id < len(colors):
                boundary_marker = self.create_line_marker(
                    area.boundary_points,
                    colors[area_id],
                    "area_boundary",  # 공통 네임스페이스
                    area_id  # 고유 ID 부여
                )
                markers.markers.append(boundary_marker)
        
        return markers
    
    def create_open_area_marker(self, open_area: ParkingArea) -> Marker:
        """열린 구역을 특별히 하이라이트하는 마커 생성"""
        return self.create_filled_area_marker(
            open_area.boundary_points,
            (0.0, 1.0, 0.0, 0.3),  # 반투명 초록색
            "open_area_fill",
            0  # 고유 ID 부여
        )
    
    def create_virtual_wall_markers(self) -> MarkerArray:
        """가상 벽을 시각화하는 마커 생성"""
        markers = MarkerArray()
        
        for i, wall in enumerate(self.virtual_walls):
            # 벽을 직육면체로 시각화
            wall_marker = self.create_cuboid_marker(
                wall.start_point, wall.end_point, wall.width,
                (1.0, 1.0, 0.0, 0.8),  # 노란색, 반투명
                "virtual_wall",  # 공통 네임스페이스
                i  # 고유 ID 부여
            )
            markers.markers.append(wall_marker)
        
        return markers
    
    def create_area_labels(self) -> MarkerArray:
        """각 구역에 ID 라벨을 표시하는 마커 생성"""
        markers = MarkerArray()
        
        for area_id, area in enumerate(self.areas):
            label_marker = self.create_text_marker(
                area.center,
                f"Area {area_id}",
                (1.0, 1.0, 1.0, 1.0),  # 흰색
                "area_label",  # 공통 네임스페이스
                area_id  # 고유 ID 부여
            )
            markers.markers.append(label_marker)
        
        return markers
    
    def create_line_marker(self, points: List[Tuple[float, float]], 
                          color: Tuple[float, float, float, float], 
                          ns: str, marker_id: int = 0) -> Marker:
        """선 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)  # 영구 표시
        
        # 위치 설정
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 크기 설정
        marker.scale = Vector3()
        marker.scale.x = 0.1  # 선 두께
        
        # 색상 설정
        marker.color = ColorRGBA()
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # 점들 추가
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            marker.points.append(p)
        
        return marker
    
    def create_filled_area_marker(self, points: List[Tuple[float, float]], 
                                 color: Tuple[float, float, float, float], 
                                 ns: str, marker_id: int = 0) -> Marker:
        """채워진 영역 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)  # 영구 표시
        
        # 위치 설정
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 크기 설정
        marker.scale = Vector3()
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # 색상 설정
        marker.color = ColorRGBA()
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # 삼각형 조각으로 나누어 점들 추가
        if len(points) >= 3:
            # 기준점 v0 = points[0]
            v0 = points[0]
            for i in range(1, len(points) - 1):
                v1 = points[i]
                v2 = points[i + 1]
                # 삼각형 (v0, v1, v2) 추가
                for vx, vy in (v0, v1, v2):
                    p = Point()
                    p.x = vx
                    p.y = vy
                    p.z = 0.0
                    marker.points.append(p)
        
        return marker
    
    def create_text_marker(self, position: Tuple[float, float], 
                          text: str, 
                          color: Tuple[float, float, float, float], 
                          ns: str, marker_id: int = 0) -> Marker:
        """텍스트 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)  # 영구 표시
        
        # 위치 설정
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        
        # 크기 설정
        marker.scale = Vector3()
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.5
        
        # 색상 설정
        marker.color = ColorRGBA()
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # 텍스트 설정
        marker.text = text
        
        return marker
    
    def create_cuboid_marker(self, start_point: Tuple[float, float], 
                            end_point: Tuple[float, float], 
                            width: float,
                            color: Tuple[float, float, float, float], 
                            ns: str, marker_id: int = 0) -> Marker:
        """직육면체 벽 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)  # 영구 표시
        
        # 벽의 중점 계산
        center_x = (start_point[0] + end_point[0]) / 2
        center_y = (start_point[1] + end_point[1]) / 2
        
        # 위치 설정
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.25  # 높이의 절반
        
        # 벽의 방향 계산 (z축 회전)
        wall_angle = np.arctan2(end_point[1] - start_point[1], 
                               end_point[0] - start_point[0])
        
        # 쿼터니언으로 변환
        from geometry_msgs.msg import Quaternion
        import math
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(wall_angle / 2.0)
        q.w = math.cos(wall_angle / 2.0)
        marker.pose.orientation = q
        
        # 크기 설정 (길이, 폭, 높이)
        wall_length = np.sqrt(
            (end_point[0] - start_point[0])**2 +
            (end_point[1] - start_point[1])**2
        )
        marker.scale = Vector3()
        marker.scale.x = wall_length  # 길이
        marker.scale.y = width         # 폭
        marker.scale.z = 0.5           # 높이
        
        # 색상 설정
        marker.color = ColorRGBA()
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        return marker
    
    def _create_and_publish_simple_visualization(self, x: float, y: float, slot_id: int):
        """단순 모드를 위한 시각화 생성 및 퍼블리시"""
        markers = MarkerArray()
        
        # 주차 포즈 마커 (큰 화살표)
        pose_marker = Marker()
        pose_marker.header.frame_id = "map"
        pose_marker.header.stamp = self.get_clock().now().to_msg()
        pose_marker.ns = "simple_parking_pose"
        pose_marker.id = 0
        pose_marker.type = Marker.ARROW
        pose_marker.action = Marker.ADD
        pose_marker.lifetime = Duration(sec=0, nanosec=0)  # 영구 표시
        
        # 위치 설정
        pose_marker.pose.position.x = x
        pose_marker.pose.position.y = y
        pose_marker.pose.position.z = 0.5
        pose_marker.pose.orientation.w = 1.0
        
        # 크기 설정 (큰 화살표)
        pose_marker.scale.x = 2.0  # 길이
        pose_marker.scale.y = 0.5  # 폭
        pose_marker.scale.z = 0.5  # 높이
        
        # 색상 설정 (밝은 녹색)
        pose_marker.color.r = 0.0
        pose_marker.color.g = 1.0
        pose_marker.color.b = 0.0
        pose_marker.color.a = 0.8
        
        markers.markers.append(pose_marker)
        
        # 구역 텍스트 마커
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "simple_slot_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.lifetime = Duration(sec=0, nanosec=0)  # 영구 표시
        
        # 위치 설정 (주차 포즈 위쪽)
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 1.5
        text_marker.pose.orientation.w = 1.0
        
        # 크기 설정
        text_marker.scale.z = 0.8
        
        # 색상 설정 (밝은 노란색)
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        
        # 텍스트 내용
        text_marker.text = f"SLOT {slot_id}\nOPEN"
        
        markers.markers.append(text_marker)
        
        # 시각화 퍼블리시
        self.visualization_pub.publish(markers)
        self.get_logger().info(f"🎨 단순 시각화 퍼블리시: 슬롯 {slot_id}, 위치 ({x:.2f}, {y:.2f})")
    
    def _try_pattern_matching(self, cone_obstacles: ObstacleArray) -> Optional[Tuple[str, int, Tuple[float, float]]]:
        """패턴 매칭을 시도하여 구역 인식"""
        min_cones = self.get_parameter('pattern.min_cones').get_parameter_value().integer_value
        if len(cone_obstacles.obstacles) < min_cones:
            self.get_logger().debug(f"패턴 매칭: 콘 개수 부족 ({len(cone_obstacles.obstacles)}/{min_cones})")
            return None
        
        # 정규화된 좌표를 사용 (패턴이 정규화된 좌표 기준이므로)
        cone_positions = []
        for obs in cone_obstacles.obstacles:
            cone_positions.append((obs.center.x, obs.center.y))
        
        self.get_logger().info(f"🔍 패턴 매칭 시도: {len(cone_positions)}개 콘")
        
        # 기존 패턴 매칭 로직 사용
        detected_pattern = self.detect_layout_pattern(cone_positions)
        if detected_pattern is None:
            self.get_logger().info("❌ 패턴 매칭 실패")
            return None
        
        # 패턴에서 열린 슬롯 정보 추출
        pattern_data = self.LAYOUT_PATTERNS[detected_pattern]
        open_slot = pattern_data["open_slot"]
        
        # 열린 슬롯의 중심 위치 계산 (정규화된 좌표 기준)
        if open_slot == 0:
            center_pos = (1.25, 2.5)  # 아래쪽 구역 중심
        elif open_slot == 1:
            center_pos = (1.25, 7.5)  # 중간 구역 중심  
        else:  # open_slot == 2
            center_pos = (1.25, 12.5) # 위쪽 구역 중심
        
        return detected_pattern, open_slot, center_pos
    
    def _publish_pattern_result(self, center_pos: Tuple[float, float], slot_id: int, pattern_name: str):
        """패턴 매칭 결과 퍼블리시"""
        x, y = center_pos
        
        # 1. 열린 구역 ID 퍼블리시
        area_msg = Int32()
        area_msg.data = slot_id
        self.open_area_pub.publish(area_msg)
        
        # 2. 정규좌표계를 map 좌표계로 변환
        map_x, map_y = self._transform_normalized_to_map(x, y)
        
        # 3. 주차 포즈 생성 (변환된 좌표 기준)
        pose = PoseStamped()
        pose.header.frame_id = "map"  # map 좌표계 사용
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x + 1.0  # 1m 앞쪽 오프셋
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        # 4. 결과 퍼블리시
        self.open_slot_pose_pub.publish(pose)
        self.open_slot_pose_stable_pub.publish(pose)
        
        # 5. 시각화
        self._create_and_publish_simple_visualization(pose.pose.position.x, pose.pose.position.y, slot_id)
        
        self.get_logger().info(f"✅ 패턴 매칭 결과: {pattern_name}, 슬롯 {slot_id}, map 좌표 ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.get_logger().info(f"   (정규좌표: ({x:.2f}, {y:.2f}) → map: ({map_x:.2f}, {map_y:.2f}))")

    # ──── 통계적 안정성 기반 구역 판단 ───────────────────────────────────────────────
    
    def _update_cone_statistics(self, cone_obstacles: ObstacleArray):
        """콘 통계 정보 업데이트"""
        if not self.statistical_enable:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for obs in cone_obstacles.obstacles:
            cone_id = obs.id
            x, y = obs.center.x, obs.center.y
            
            # 콘 히스토리에 추가
            if cone_id not in self._cone_history:
                self._cone_history[cone_id] = []
            
            self._cone_history[cone_id].append({
                'x': x, 'y': y, 't': current_time, 'confidence': 1.0
            })
            
            # 히스토리 길이 제한
            if len(self._cone_history[cone_id]) > self.statistical_max_history:
                self._cone_history[cone_id] = self._cone_history[cone_id][-self.statistical_max_history:]
        
        # 안정적인 콘 업데이트
        self._update_stable_cones(current_time)
    
    def _update_stable_cones(self, current_time: float):
        """안정적인 콘들 업데이트"""
        for cone_id, history in self._cone_history.items():
            if len(history) < self.statistical_min_frames:
                continue
                
            # 최근 프레임들만 고려
            recent_frames = [h for h in history if current_time - h['t'] < 5.0]  # 5초 이내
            if len(recent_frames) < self.statistical_min_frames:
                continue
            
            # 위치 안정성 검사
            positions = [(h['x'], h['y']) for h in recent_frames]
            avg_x = sum(p[0] for p in positions) / len(positions)
            avg_y = sum(p[1] for p in positions) / len(positions)
            
            # 위치 분산 계산
            variance = sum((p[0] - avg_x)**2 + (p[1] - avg_y)**2 for p in positions) / len(positions)
            std_dev = math.sqrt(variance)
            
            # 안정성 기준 만족시 안정적인 콘으로 등록
            if std_dev < self.statistical_position_tolerance:
                confidence = min(1.0, len(recent_frames) / self.statistical_min_frames)
                self._stable_cones[cone_id] = {
                    'x': avg_x, 'y': avg_y, 'confidence': confidence, 'last_seen': current_time
                }
                self.get_logger().debug(f"📊 안정적인 콘 등록: ID={cone_id}, 위치=({avg_x:.2f}, {avg_y:.2f}), 신뢰도={confidence:.2f}")
    
    def _update_vehicle_trajectory(self, vehicle_pose: Tuple[float, float, float]):
        """차량 궤적 업데이트"""
        if not self.trajectory_enable:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        x, y, yaw = vehicle_pose
        
        self._vehicle_poses.append({
            'x': x, 'y': y, 'yaw': yaw, 't': current_time
        })
        
        # 궤적 히스토리 길이 제한
        if len(self._vehicle_poses) > 100:  # 최대 100개 포인트
            self._vehicle_poses = self._vehicle_poses[-100:]
        
        self._current_vehicle_pose = {'x': x, 'y': y, 'yaw': yaw, 't': current_time}
    
    def _detect_parking_area_statistical(self) -> Optional[Tuple[int, Tuple[float, float]]]:
        """통계적 안정성 기반 주차 구역 감지"""
        if not self.statistical_enable or not self._stable_cones:
            return None
        
        # 신뢰도가 높은 콘들만 필터링
        high_confidence_cones = {
            cone_id: cone_data for cone_id, cone_data in self._stable_cones.items()
            if cone_data['confidence'] >= self.statistical_confidence_threshold
        }
        
        if len(high_confidence_cones) < 3:
            self.get_logger().debug(f"📊 통계적 감지: 신뢰도 높은 콘 부족 ({len(high_confidence_cones)}/3)")
            return None
        
        # 차량 궤적 기반 필터링 (활성화된 경우)
        if self.trajectory_enable and self._current_vehicle_pose:
            filtered_cones = self._filter_cones_by_trajectory(high_confidence_cones)
            if len(filtered_cones) < 3:
                self.get_logger().debug(f"📊 궤적 필터링: 수직 콘 부족 ({len(filtered_cones)}/3)")
                return None
        else:
            filtered_cones = high_confidence_cones
        
        # 주차 구역 감지 로직 (간단한 갭 기반)
        return self._find_parking_gap_statistical(filtered_cones)
    
    def _filter_cones_by_trajectory(self, cones: dict) -> dict:
        """차량 궤적에 수직인 콘들만 필터링"""
        if not self._current_vehicle_pose or len(self._vehicle_poses) < 2:
            return cones
        
        # 차량 진행 방향 계산 (최근 2개 포인트 기준)
        recent_poses = self._vehicle_poses[-2:]
        if len(recent_poses) < 2:
            return cones
        
        dx = recent_poses[-1]['x'] - recent_poses[-2]['x']
        dy = recent_poses[-1]['y'] - recent_poses[-2]['y']
        vehicle_direction = math.atan2(dy, dx)
        
        filtered_cones = {}
        vehicle_x, vehicle_y = self._current_vehicle_pose['x'], self._current_vehicle_pose['y']
        
        for cone_id, cone_data in cones.items():
            cone_x, cone_y = cone_data['x'], cone_data['y']
            
            # 콘과 차량 간 거리
            distance = math.sqrt((cone_x - vehicle_x)**2 + (cone_y - vehicle_y)**2)
            if distance < self.trajectory_min_cone_distance:
                continue
            
            # 콘 방향 벡터
            cone_dx = cone_x - vehicle_x
            cone_dy = cone_y - vehicle_y
            cone_direction = math.atan2(cone_dy, cone_dx)
            
            # 수직성 검사
            angle_diff = abs(cone_direction - vehicle_direction)
            if angle_diff > math.pi/2:
                angle_diff = math.pi - angle_diff
            
            if angle_diff < self.trajectory_perpendicular_tolerance:
                filtered_cones[cone_id] = cone_data
                self.get_logger().debug(f"📊 수직 콘 감지: ID={cone_id}, 각도차={math.degrees(angle_diff):.1f}°")
        
        return filtered_cones
    
    def _find_parking_gap_statistical(self, cones: dict) -> Optional[Tuple[int, Tuple[float, float]]]:
        """통계적 안정성 기반 갭 감지"""
        if len(cones) < 2:
            return None
        
        # 콘들을 Y 좌표로 정렬
        sorted_cones = sorted(cones.items(), key=lambda x: x[1]['y'])
        
        # 가장 큰 갭 찾기
        max_gap = 0
        gap_center = None
        gap_slot = 0
        
        for i in range(len(sorted_cones) - 1):
            cone1_y = sorted_cones[i][1]['y']
            cone2_y = sorted_cones[i + 1][1]['y']
            gap_size = cone2_y - cone1_y
            
            if gap_size > max_gap and gap_size >= self.gap_min_length:
                max_gap = gap_size
                gap_center = (
                    (sorted_cones[i][1]['x'] + sorted_cones[i + 1][1]['x']) / 2,
                    (cone1_y + cone2_y) / 2
                )
                gap_slot = i  # 간단한 슬롯 ID
        
        if gap_center:
            self.get_logger().info(f"📊 통계적 갭 감지: 크기={max_gap:.2f}m, 중심=({gap_center[0]:.2f}, {gap_center[1]:.2f})")
            return gap_slot, gap_center
        
        return None
    
    def _publish_statistical_result(self, slot_id: int, center_pos: Tuple[float, float]):
        """통계적 안정성 결과 퍼블리시"""
        x, y = center_pos
        
        # 1. 열린 구역 ID 퍼블리시
        area_msg = Int32()
        area_msg.data = slot_id
        self.open_area_pub.publish(area_msg)
        
        # 2. 정규좌표계를 map 좌표계로 변환
        map_x, map_y = self._transform_normalized_to_map(x, y)
        
        # 3. 주차 포즈 생성
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x + 1.0  # 1m 앞쪽 오프셋
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        # 4. 결과 퍼블리시
        self.open_slot_pose_pub.publish(pose)
        self.open_slot_pose_stable_pub.publish(pose)
        
        # 5. 시각화
        self._create_and_publish_simple_visualization(pose.pose.position.x, pose.pose.position.y, slot_id)
        
        self.get_logger().info(f"✅ 통계적 안정성 결과: 슬롯 {slot_id}, map 좌표 ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.get_logger().info(f"   (정규좌표: ({x:.2f}, {y:.2f}) → map: ({map_x:.2f}, {map_y:.2f}))")
    
    def process_vehicle_pose(self, pose_msg: PoseStamped):
        """차량 위치 처리 (궤적 분석용)"""
        if not self.trajectory_enable:
            return
        
        # 위치 추출
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        
        # 방향 추출 (쿼터니언에서 yaw 각도)
        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w
        
        # 쿼터니언을 yaw 각도로 변환
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        
        # 궤적 업데이트
        self._update_vehicle_trajectory((x, y, yaw))


def main(args=None):
    rclpy.init(args=args)
    
    parking_area_node = ParkingAreaNode()
    
    try:
        rclpy.spin(parking_area_node)
    except KeyboardInterrupt:
        pass
    finally:
        parking_area_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
