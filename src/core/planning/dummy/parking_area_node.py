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
        self.declare_parameter('slot_origin_x', 0.0)      # X 기준선 (도로 가장자리)
        self.declare_parameter('slot_origin_y', 0.0)      # 가장 아래 슬롯의 y 하단
        self.declare_parameter('SLOT_LEN', 5.0)           # L : 세로 길이
        self.declare_parameter('SLOT_GAP', 0.01)           # G : 슬롯 간 간격
        self.declare_parameter('SLOT_WIDTH', 2.5)         # W : 가로 폭
        self.declare_parameter('EPS_X', 0.3)              # 좌/우 여유 허용치
        self.declare_parameter('EPS_Y', 0.01)              # 상/하 여유 허용치
        
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
        
        # TF 브로드캐스터 추가
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
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
        
        # 크기 정보 업데이트
        self.SLOT_WIDTH = max_x - min_x
        total_length = max_y - min_y
        self.SLOT_LEN = total_length / 3.0
        
        # 변경사항 로그
        if (abs(old_origin_x - self.slot_origin_x) > 0.1 or 
            abs(old_origin_y - self.slot_origin_y) > 0.1):
            self.get_logger().info('📍 실제 콘 위치로 파라미터 조정:')
            self.get_logger().info(f'  기준점: ({old_origin_x:.2f}, {old_origin_y:.2f}) → ({self.slot_origin_x:.2f}, {self.slot_origin_y:.2f})')
            self.get_logger().info(f'  크기: {self.SLOT_WIDTH:.2f} × {total_length:.2f}')
    
    def process_cones(self, cone_obstacles: ObstacleArray):
        """라바콘 위치 정보를 처리하여 구역을 정의하고 분석"""
        # 라바콘 위치 추출
        cone_positions = [(obs.center.x, obs.center.y) for obs in cone_obstacles.obstacles]
        
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
    
    # ──── ❺ 열린 슬롯 탐색 (핵심 로직) ───────────────────────────────────────────────
    def find_open_slot(self, cones: List[Tuple[float, float]]) -> int:
        """열린 슬롯 탐색"""
        for sid in [0, 1, 2]:  # 아래→위 순서
            if self.left_cone_count(cones, sid) < 3:  # 0,1,2개 → open
                return sid
        return -1  # 예외: 모든 슬롯이 콘 3개 → 전부 닫힘
    
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
        seg = [(3, 2), (2, 1)]     # 위쪽·오른쪽만 (좌측 개방)
        walls = []
        
        for i, j in seg:
            start = bp[i]
            end = bp[j]
            length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            width = min(length * 0.1, 0.2)
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
            
            length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            width = min(length * 0.1, 0.2)
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
            obstacle.type = "virtual_wall"
            
            # 벽의 중점을 중심으로 설정
            center_x = (wall.start_point[0] + wall.end_point[0]) / 2
            center_y = (wall.start_point[1] + wall.end_point[1]) / 2
            
            obstacle.center.x = center_x
            obstacle.center.y = center_y
            obstacle.center.z = 0.0
            
            # 벽의 폭을 반지름으로 설정 (충돌 검사용)
            obstacle.radius = wall.width / 2
            
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
