#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, PoseStamped, Pose, Quaternion, TransformStamped
from std_msgs.msg import ColorRGBA, Int32
from planning_msgs.msg import ObstacleArray, Obstacle
from builtin_interfaces.msg import Duration
import tf2_ros
import numpy as np
import math
from typing import List, Tuple



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
        self.declare_parameter('align.use_line_only', True)  # 도로측 일렬 콘만으로 템플릿 정렬(L만 갱신, W는 고정 유지).
        self.declare_parameter('align.min_span_y', 1.0)  # 일렬 관측 y-스팬 최소 보장값(너무 짧을 때 L 산정 하한).
        self.declare_parameter('align.lock_width', True)  # 관측 폭이 작거나 라인만 관측 시 W(폭) 고정 유지
        self.declare_parameter('align.min_width_span', 1.0)  # 폭 갱신을 허용하는 최소 x 스팬
        self.declare_parameter('align.disable', True)    # 템플릿 얼라인 완전 비활성(좌표계 축 고정)
        # Sequential mode parameters
        # time/clock
        self.declare_parameter('seq.buffer_window_sec', 1.0)  # 최근 이 시간(초) 윈도의 콘만 버퍼링(부분 관측 누적 창 크기).
        self.declare_parameter('seq.cone_timeout_sec', 10000000000.0)   # 캐시된 콘의 유효 시간(초). 초과 시 제거합니다.
        self.declare_parameter('seq.confirm_threshold', 0.1)  # EMA 점수 확정 임계값. 높을수록 보수적으로 확정합니다.
        self.declare_parameter('seq.release_threshold', 0.9)  # EMA 점수 해제 임계값. 낮을수록 상태 유지가 강합니다.
        self.declare_parameter('seq.stable_hold_sec', 0.8)    # 확정 후 유지해야 하는 최소 시간(깜빡임 방지).
        self.declare_parameter('seq.ema_alpha', 0.5)          # EMA 가중치. 높을수록 최신 관측 반영이 큽니다.
        self.declare_parameter('seq.publish_stable_only', False)  # True면 안정 결과만 퍼블리시(실시간 변동 억제).
        self.declare_parameter('seq.logging_enabled', True)       # 시퀀스 로그 기록 on/off.
        self.declare_parameter('seq.log_path', '/tmp/parking_area_seq.csv')  # 시퀀스 로그 파일 경로.
        self.declare_parameter('seq.map_cache_ttl_sec', 5.0)      # 라벨/맵 캐시 TTL(초). 오래된 항목을 자동 삭제합니다.
        # Robustness controls against frame drops / oscillation
        self.declare_parameter('seq.skip_on_drop', False)      # 버퍼 크기 급감 시 업데이트 건너뜀(진동 억제).
        self.declare_parameter('seq.min_score_delta', 0.08)    # 모드 전환 최소 점수 차이. 클수록 전환 둔감.
        self.declare_parameter('seq.provisional_dwell_sec', 0.4)  # 임시 우승 모드 유지 필요 시간(전환 안정화).
        self.declare_parameter('seq.release_grace_sec', 0.8)      # 해제 조건 지속 필요 시간(불안정 기간 허용).
        # Road-side line extraction params
        self.declare_parameter('line.quantile', 0.2)           # 일렬 대표 x 추정 시 좌측 분위수(도로측 선택 정도).
        self.declare_parameter('line.x_tolerance', 0.35)       # 대표 x 주변 허용 오차(m). 일렬 포인트 판정 폭.
        self.declare_parameter('line.min_points', 3)           # 일렬로 인정할 최소 포인트 수.
        # PCA/RANSAC line fit params
        self.declare_parameter('line.use_pca', True)           # PCA로 주성분(도로측 라인) 추정 사용
        self.declare_parameter('line.distance_tolerance', 0.25) # 라인으로부터 허용 거리(일렬로 채택 기준)
        self.declare_parameter('line.use_ransac', True)       # RANSAC 사용 여부(옵션)
        # Gap-based open-slot detection
        self.declare_parameter('gap.use', True)                # s-축 연속 점들 간 큰 간격을 열린 구역으로 판정
        self.declare_parameter('gap.min_length', 4)          # 열린 구역으로 간주할 최소 갭 길이(m)
        self.declare_parameter('gap.pick_strategy', 'largest') # largest | first
        # Simple mode (경량 판정 전용 경로)
        self.declare_parameter('simple.enabled', False)        # True면 단순 파이프라인만 수행
        self.declare_parameter('simple.side', 'right')         # right|left|all
        self.declare_parameter('simple.gap_min', 1.2)          # 갭 기반 열린 슬롯 임계(m)
        self.declare_parameter('simple.width', 2.5)            # 슬롯 폭 가정(중심 오프셋 계산용)
        self.declare_parameter('simple.distance_tolerance', 0.25)  # 라인 법선 허용 거리
        # Persistent cone map (미션 종료까지 보존)
        self.declare_parameter('persist.enable', True)          # 영속 맵 사용 여부
        self.declare_parameter('persist.merge_radius', 0.1)     # 같은 콘으로 병합하는 반경(m)
        self.declare_parameter('persist.use_for_decision', True) # 의사결정에 영속 맵 우선 사용
        self.declare_parameter('persist.freeze_after_confirm', False) # 안정 확정 후 갱신 정지
        self.declare_parameter('persist.reset_topic', '/parking/reset_map') # 맵 리셋 토픽(Empty)
        
        # 파라미터 로드
        self.slot_origin_x = self.get_parameter('slot_origin_x').value
        self.slot_origin_y = self.get_parameter('slot_origin_y').value
        self.SLOT_LEN = self.get_parameter('SLOT_LEN').value
        self.SLOT_GAP = self.get_parameter('SLOT_GAP').value
        self.SLOT_WIDTH = self.get_parameter('SLOT_WIDTH').value
        self.EPS_X = self.get_parameter('EPS_X').value
        self.EPS_Y = self.get_parameter('EPS_Y').value
        
        # 구독자
        self.cone_sub = self.create_subscription(
            ObstacleArray,
            '/cones_obstacles',
            self.process_cones,
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
    
    def process_cones(self, cone_obstacles: ObstacleArray):
        """라바콘 위치 정보를 처리하여 구역을 정의하고 분석"""
        # 경량 모드: 가장 단순한 라인+갭 기반 판정만 수행
        if bool(self.get_parameter('simple.enabled').value):
            try:
                self.process_cones_simple(cone_obstacles)
            except Exception:
                pass
            return
        mode = str(self.get_parameter('recognition_mode').value).lower()
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
        cones = []
        for obs in cone_obstacles.obstacles:
            desc = (obs.description or '').lower().strip() if hasattr(obs, 'description') else ''
            if side == 'right' and desc and desc != 'right':
                continue
            if side == 'left' and desc and desc != 'left':
                continue
            cones.append((float(obs.center.x), float(obs.center.y)))
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
        self.publish_open_area_id()
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
            # 차량 기준 오른쪽 콘만 유지(상위 필터와 일치)
            try:
                if hasattr(obs, 'description') and obs.description:
                    if str(obs.description).strip().lower() != 'right':
                        continue
            except Exception:
                pass
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
        # Update cache and build buffered cones
        self._seq_update_cache(cone_obstacles)
        # Persistent update (no TTL)
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
        k = int(self.declare_parameter('slot.cones_per_closed_band', 3).value)
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
        """구역 객체 채우기"""
        areas = []
        
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
