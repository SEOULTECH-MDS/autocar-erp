#!/usr/bin/env python3
"""
콘 맵 노드 - 영구적인 콘 저장 및 관리

기능:
1. 어댑터로부터 콘 관측 데이터 수집
2. 중복 콘 제거 및 병합
3. 구역 판단용 정규화된 좌표계 생성
4. 라벨링 가능한 콘 맵 퍼블리싱
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from collections import defaultdict

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header, Int32
from visualization_msgs.msg import MarkerArray, Marker
from planning_msgs.msg import ObstacleArray, Obstacle, ConeMapMsg, ConeInfo as ConeInfoMsg


@dataclass
class ConeInfo:
    """콘 정보 저장 클래스"""
    id: int
    position: Tuple[float, float]  # (x, y) in normalized coordinate
    original_position: Tuple[float, float]  # (x, y) in map frame
    side_label: str  # "left", "right", "center"
    confidence: float = 1.0
    observation_count: int = 1
    last_seen: float = 0.0
    
    def __post_init__(self):
        self.last_seen = rclpy.clock.Clock().now().nanoseconds / 1e9


@dataclass
class ConeMapState:
    """콘 맵 상태 관리"""
    cones: Dict[int, ConeInfo] = field(default_factory=dict)
    coordinate_system_ready: bool = False
    origin: Tuple[float, float] = (0.0, 0.0)
    rotation_angle: float = 0.0  # radians
    
    def add_cone(self, cone_info: ConeInfo) -> bool:
        """콘 추가/업데이트"""
        existing_cone = self._find_nearby_cone(cone_info.original_position)
        
        if existing_cone:
            # 기존 콘 업데이트
            existing_cone.confidence += 0.5
            existing_cone.observation_count += 1
            existing_cone.last_seen = cone_info.last_seen
            return False  # 새 콘이 아님
        else:
            # 새 콘 추가
            self.cones[cone_info.id] = cone_info
            return True  # 새 콘 추가됨
    
    def _find_nearby_cone(self, position: Tuple[float, float], threshold: float = 0.5) -> Optional[ConeInfo]:
        """주변에 있는 기존 콘 찾기"""
        for cone in self.cones.values():
            dist = math.sqrt(
                (cone.original_position[0] - position[0])**2 + 
                (cone.original_position[1] - position[1])**2
            )
            if dist < threshold:
                return cone
        return None
    
    def setup_coordinate_system(self) -> bool:
        """정규화된 좌표계 설정"""
        if len(self.cones) < 3:
            return False
        
        # 가장 신뢰도 높은 콘들로 라인 피팅
        reliable_cones = sorted(
            self.cones.values(), 
            key=lambda c: c.confidence * c.observation_count, 
            reverse=True
        )[:min(5, len(self.cones))]
        
        if len(reliable_cones) < 2:
            return False
        
        # PCA로 주 방향 계산
        positions = np.array([c.original_position for c in reliable_cones])
        
        # 원점을 첫 번째 신뢰도 높은 콘으로 설정
        self.origin = reliable_cones[0].original_position
        
        # 중심점 계산
        center = np.mean(positions, axis=0)
        centered_pos = positions - center
        
        # PCA
        if len(centered_pos) > 1:
            cov_matrix = np.cov(centered_pos.T)
            eigenvals, eigenvecs = np.linalg.eig(cov_matrix)
            
            # 주성분 방향을 회전 각도로 설정
            main_direction = eigenvecs[:, np.argmax(eigenvals)]
            self.rotation_angle = math.atan2(main_direction[1], main_direction[0])
        
        # 모든 콘의 정규화된 좌표 계산
        self._update_normalized_coordinates()
        
        self.coordinate_system_ready = True
        return True
    
    def _update_normalized_coordinates(self):
        """모든 콘의 정규화된 좌표 업데이트"""
        cos_angle = math.cos(-self.rotation_angle)
        sin_angle = math.sin(-self.rotation_angle)
        
        for cone in self.cones.values():
            # 원점 기준으로 이동
            x = cone.original_position[0] - self.origin[0]
            y = cone.original_position[1] - self.origin[1]
            
            # 회전 변환
            rotated_x = x * cos_angle - y * sin_angle
            rotated_y = x * sin_angle + y * cos_angle
            
            cone.position = (rotated_x, rotated_y)


class ConeMapNode(Node):
    """콘 맵 관리 노드"""
    
    def __init__(self):
        super().__init__('cone_map_node')
        
        # 파라미터 선언
        self.declare_parameter('input_topic', '/cones_obstacles')
        self.declare_parameter('output_topic', '/cone_map')
        self.declare_parameter('visualization_topic', '/cone_map_markers')
        self.declare_parameter('merge_distance_threshold', 0.5)
        self.declare_parameter('min_cones_for_coordinate_system', 3)
        self.declare_parameter('confidence_threshold', 2.0)
        self.declare_parameter('max_cones', 50)
        self.declare_parameter('publish_rate_hz', 2.0)
        
        # 상태 초기화
        self.cone_map = ConeMapState()
        self.next_cone_id = 1
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 구독자/발행자 설정
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.cone_subscription = self.create_subscription(
            ObstacleArray,
            input_topic,
            self.cone_callback,
            qos_profile
        )
        
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.cone_map_publisher = self.create_publisher(
            ConeMapMsg,
            output_topic,
            qos_profile
        )
        
        viz_topic = self.get_parameter('visualization_topic').get_parameter_value().string_value
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            viz_topic,
            qos_profile
        )
        
        # 주기적 퍼블리싱 타이머
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_cone_map)
        
        self.get_logger().info("🗺️ 콘 맵 노드 시작됨")
        self.get_logger().info(f"   입력: {input_topic}")
        self.get_logger().info(f"   출력: {output_topic}")
        self.get_logger().info(f"   시각화: {viz_topic}")
    
    def cone_callback(self, msg: ObstacleArray):
        """콘 관측 데이터 수신 콜백"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        new_cones_count = 0
        
        for obs in msg.obstacles:
            # 콘 정보 생성
            cone_info = ConeInfo(
                id=self.next_cone_id,
                position=(0.0, 0.0),  # 나중에 정규화
                original_position=(float(obs.center.x), float(obs.center.y)),
                side_label=getattr(obs, 'description', 'unknown'),
                last_seen=current_time
            )
            
            # 콘 맵에 추가
            if self.cone_map.add_cone(cone_info):
                new_cones_count += 1
                self.next_cone_id += 1
        
        # 좌표계 업데이트 시도
        if not self.cone_map.coordinate_system_ready:
            min_cones = self.get_parameter('min_cones_for_coordinate_system').get_parameter_value().integer_value
            if len(self.cone_map.cones) >= min_cones:
                if self.cone_map.setup_coordinate_system():
                    self.get_logger().info(f"🎯 좌표계 설정 완료: 원점=({self.cone_map.origin[0]:.2f}, {self.cone_map.origin[1]:.2f}), 회전={math.degrees(self.cone_map.rotation_angle):.1f}°")
                    
                    # 정규화 강제 재계산 (기존 _update_normalized_coordinates가 실패한 경우를 위해)
                    self.get_logger().info("🔧 정규화 좌표 강제 재계산 시작...")
                    updated_count = 0
                    for cone_id, cone in self.cone_map.cones.items():
                        if cone.original_position[0] != 0.0 or cone.original_position[1] != 0.0:  # 유효한 원본 좌표만
                            # 원점 기준으로 이동
                            x = cone.original_position[0] - self.cone_map.origin[0]
                            y = cone.original_position[1] - self.cone_map.origin[1]
                            
                            # 회전 변환
                            cos_angle = math.cos(-self.cone_map.rotation_angle)
                            sin_angle = math.sin(-self.cone_map.rotation_angle)
                            rotated_x = x * cos_angle - y * sin_angle
                            rotated_y = x * sin_angle + y * cos_angle
                            
                            # 업데이트
                            cone.position = (rotated_x, rotated_y)
                            updated_count += 1
                    
                    # 결과 확인
                    non_zero_count = sum(1 for c in self.cone_map.cones.values() 
                                       if abs(c.position[0]) > 0.01 or abs(c.position[1]) > 0.01)
                    self.get_logger().info(f"✅ 정규화 완료: {non_zero_count}/{len(self.cone_map.cones)}개 콘이 정규화됨")
        
        if new_cones_count > 0:
            self.get_logger().info(f"🔍 새 콘 {new_cones_count}개 추가, 총 {len(self.cone_map.cones)}개 콘 저장")
            
            # 좌표계가 이미 준비된 상태에서 새로 추가된 콘들 정규화
            if self.cone_map.coordinate_system_ready:
                self.get_logger().info(f"🔧 새 콘 {new_cones_count}개 정규화 시작...")
                normalized_count = 0
                for cone_id, cone in self.cone_map.cones.items():
                    if (cone.position == (0.0, 0.0) and 
                        (cone.original_position[0] != 0.0 or cone.original_position[1] != 0.0)):
                        # 원점 기준으로 이동
                        x = cone.original_position[0] - self.cone_map.origin[0]
                        y = cone.original_position[1] - self.cone_map.origin[1]
                        
                        # 회전 변환
                        cos_angle = math.cos(-self.cone_map.rotation_angle)
                        sin_angle = math.sin(-self.cone_map.rotation_angle)
                        rotated_x = x * cos_angle - y * sin_angle
                        rotated_y = x * sin_angle + y * cos_angle
                        
                        # 업데이트
                        cone.position = (rotated_x, rotated_y)
                        normalized_count += 1
                
                if normalized_count > 0:
                    self.get_logger().info(f"✅ 새 콘 정규화 완료: {normalized_count}개")
    
    def publish_cone_map(self):
        """콘 맵 퍼블리싱"""
        if not self.cone_map.cones:
            return
        
        # ConeMapMsg 메시지 생성
        cone_map_msg = ConeMapMsg()
        cone_map_msg.header.stamp = self.get_clock().now().to_msg()
        cone_map_msg.header.frame_id = 'cone_map'
        
        # 좌표계 정보
        cone_map_msg.coordinate_system_ready = self.cone_map.coordinate_system_ready
        cone_map_msg.origin.x = self.cone_map.origin[0]
        cone_map_msg.origin.y = self.cone_map.origin[1] 
        cone_map_msg.origin.z = 0.0
        cone_map_msg.rotation_angle = self.cone_map.rotation_angle
        
        # 콘 데이터
        reliable_count = 0
        for cone in self.cone_map.cones.values():
            cone_info_msg = ConeInfoMsg()
            cone_info_msg.id = cone.id
            
            # 정규화된 좌표
            cone_info_msg.position.x = cone.position[0]
            cone_info_msg.position.y = cone.position[1]
            cone_info_msg.position.z = 0.0
            
            # 원본 좌표
            cone_info_msg.original_position.x = cone.original_position[0]
            cone_info_msg.original_position.y = cone.original_position[1]
            cone_info_msg.original_position.z = 0.0
            
            # 메타데이터
            cone_info_msg.side_label = cone.side_label
            cone_info_msg.area_label = "unknown"  # 구역 판단 후 설정될 예정
            cone_info_msg.confidence = cone.confidence
            cone_info_msg.observation_count = cone.observation_count
            
            # 시간 정보
            cone_info_msg.last_seen.sec = int(cone.last_seen)
            cone_info_msg.last_seen.nanosec = int((cone.last_seen % 1.0) * 1e9)
            
            cone_map_msg.cones.append(cone_info_msg)
            
            if cone.confidence >= 2.0:
                reliable_count += 1
        
        # 메타데이터
        cone_map_msg.total_cone_count = len(self.cone_map.cones)
        cone_map_msg.reliable_cone_count = reliable_count
        cone_map_msg.last_update = self.get_clock().now().to_msg()
        
        self.cone_map_publisher.publish(cone_map_msg)
        
        # 시각화 마커 퍼블리싱
        self.publish_visualization()
    
    def publish_visualization(self):
        """콘 맵 시각화 마커 퍼블리싱"""
        marker_array = MarkerArray()
        
        for i, cone in enumerate(self.cone_map.cones.values()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'cone_map'
            marker.id = cone.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 원본 위치 사용 (map frame)
            marker.pose.position.x = cone.original_position[0]
            marker.pose.position.y = cone.original_position[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 크기
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.6
            
            # 색상 (신뢰도에 따라)
            if cone.confidence >= 3.0:
                # 고신뢰도: 녹색
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif cone.confidence >= 2.0:
                # 중신뢰도: 노란색
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                # 저신뢰도: 빨간색
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        # 좌표계 표시 (원점과 축)
        if self.cone_map.coordinate_system_ready:
            # 원점 마커
            origin_marker = Marker()
            origin_marker.header.frame_id = 'map'
            origin_marker.header.stamp = self.get_clock().now().to_msg()
            origin_marker.ns = 'cone_map_coord'
            origin_marker.id = 9999
            origin_marker.type = Marker.CUBE
            origin_marker.action = Marker.ADD
            
            origin_marker.pose.position.x = self.cone_map.origin[0]
            origin_marker.pose.position.y = self.cone_map.origin[1]
            origin_marker.pose.position.z = 0.0
            origin_marker.pose.orientation.w = 1.0
            
            origin_marker.scale.x = 0.5
            origin_marker.scale.y = 0.5
            origin_marker.scale.z = 0.1
            
            origin_marker.color.r = 0.0
            origin_marker.color.g = 0.0
            origin_marker.color.b = 1.0
            origin_marker.color.a = 0.9
            
            marker_array.markers.append(origin_marker)
        
        self.visualization_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ConeMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
  