#!/usr/bin/env python3
"""
구역 판단 노드 - 콘 맵 기반 주차 구역 감지 및 라벨링

콘 맵 노드로부터 정규화된 콘 데이터를 받아서:
1. 주차 구역 패턴 분석
2. 열린/닫힌 구역 판별
3. 각 콘에 구역 라벨 추가
4. 라벨링된 결과를 플래너로 전송
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header, Int32
from visualization_msgs.msg import MarkerArray, Marker
from planning_msgs.msg import ConeMapMsg, ConeInfo, ObstacleArray, Obstacle


class AreaStatus(Enum):
    UNKNOWN = "unknown"
    OPEN = "open" 
    CLOSED = "closed"


@dataclass
class ParkingArea:
    """주차 구역 정보"""
    area_id: int
    status: AreaStatus
    cone_ids: List[int]
    cone_positions: List[Tuple[float, float]]  # 정규화된 좌표
    center: Tuple[float, float]
    confidence: float = 0.0
    
    
class AreaDetectionNode(Node):
    """콘 맵 기반 구역 판단 노드"""
    
    def __init__(self):
        super().__init__('area_detection_node')
        
        # 파라미터 선언
        self.declare_parameter('input_topic', '/cone_map')
        self.declare_parameter('output_topic', '/labeled_cone_map')
        self.declare_parameter('open_slot_topic', '/open_slot_pose')
        self.declare_parameter('visualization_topic', '/area_detection_markers')
        
        # 구역 판단 파라미터
        self.declare_parameter('slot_width', 2.5)      # 주차 슬롯 폭
        self.declare_parameter('slot_length', 5.0)     # 주차 슬롯 길이  
        self.declare_parameter('gap_threshold', 3.0)   # 열린 구역 판단 임계값
        self.declare_parameter('min_cones_for_area', 2) # 구역 판단 최소 콘 개수
        self.declare_parameter('y_tolerance', 1.0)     # Y축 허용 오차
        
        # 상태 초기화
        self.parking_areas: List[ParkingArea] = []
        self.last_cone_map: Optional[ConeMapMsg] = None
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 구독자/발행자 설정
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.cone_map_subscription = self.create_subscription(
            ConeMapMsg,
            input_topic,
            self.cone_map_callback,
            qos_profile
        )
        
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.labeled_cone_publisher = self.create_publisher(
            ConeMapMsg,
            output_topic,
            qos_profile
        )
        
        slot_topic = self.get_parameter('open_slot_topic').get_parameter_value().string_value
        self.open_slot_publisher = self.create_publisher(
            PoseStamped,
            slot_topic,
            qos_profile
        )
        
        viz_topic = self.get_parameter('visualization_topic').get_parameter_value().string_value
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            viz_topic,
            qos_profile
        )
        
        self.get_logger().info("🎯 구역 판단 노드 시작됨")
        self.get_logger().info(f"   입력: {input_topic}")
        self.get_logger().info(f"   출력: {output_topic}")
        self.get_logger().info(f"   열린 슬롯: {slot_topic}")
        self.get_logger().info(f"   시각화: {viz_topic}")
    
    def cone_map_callback(self, msg: ConeMapMsg):
        """콘 맵 수신 및 구역 판단"""
        if not msg.coordinate_system_ready:
            self.get_logger().warn("⚠️ 콘 맵 좌표계가 아직 준비되지 않음")
            return
        
        self.last_cone_map = msg
        self.get_logger().info(f"🔍 콘 맵 수신: {len(msg.cones)}개 콘, 신뢰도 높은 콘: {msg.reliable_cone_count}개")
        
        # 구역 판단 수행
        self.detect_parking_areas()
        
        # 라벨링된 결과 퍼블리시
        self.publish_labeled_cone_map()
        
        # 열린 슬롯 퍼블리시
        self.publish_open_slot()
        
        # 시각화 퍼블리시
        self.publish_visualization()
    
    def detect_parking_areas(self):
        """주차 구역 감지 및 분석"""
        if not self.last_cone_map or not self.last_cone_map.cones:
            return
        
        # 신뢰도 높은 콘만 사용
        reliable_cones = [
            cone for cone in self.last_cone_map.cones 
            if cone.confidence >= 2.0
        ]
        
        if len(reliable_cones) < self.get_parameter('min_cones_for_area').get_parameter_value().integer_value:
            self.get_logger().warn(f"⚠️ 신뢰도 높은 콘이 부족: {len(reliable_cones)}개")
            return
        
        self.get_logger().info(f"📊 구역 판단용 콘: {len(reliable_cones)}개")
        
        # 정규화된 좌표계에서 라인 콘 추출 (Y=0 근처)
        y_tolerance = self.get_parameter('y_tolerance').get_parameter_value().double_value
        line_cones = [
            cone for cone in reliable_cones
            if abs(cone.position.y) <= y_tolerance
        ]
        
        # X축으로 정렬
        line_cones.sort(key=lambda c: c.position.x)
        
        self.get_logger().info(f"🔍 라인 콘 추출: {len(line_cones)}개")
        
        if len(line_cones) < 2:
            self.get_logger().warn("⚠️ 라인 콘이 부족하여 구역 판단 불가")
            return
        
        # 갭 기반 구역 감지
        self.parking_areas = self._detect_areas_by_gaps(line_cones)
        
        self.get_logger().info(f"✅ 감지된 구역: {len(self.parking_areas)}개")
        for area in self.parking_areas:
            self.get_logger().info(f"   구역 {area.area_id}: {area.status.value}, 콘 {len(area.cone_ids)}개, 중심=({area.center[0]:.2f}, {area.center[1]:.2f})")
    
    def _detect_areas_by_gaps(self, line_cones: List[ConeInfo]) -> List[ParkingArea]:
        """갭 기반 구역 감지"""
        areas = []
        gap_threshold = self.get_parameter('gap_threshold').get_parameter_value().double_value
        
        for i in range(len(line_cones) - 1):
            cone1 = line_cones[i]
            cone2 = line_cones[i + 1]
            
            # 두 콘 사이의 거리 계산
            distance = abs(cone2.position.x - cone1.position.x)
            
            # 갭이 임계값보다 크면 열린 구역
            if distance >= gap_threshold:
                center_x = (cone1.position.x + cone2.position.x) / 2.0
                center_y = (cone1.position.y + cone2.position.y) / 2.0
                
                area = ParkingArea(
                    area_id=len(areas) + 1,
                    status=AreaStatus.OPEN,
                    cone_ids=[cone1.id, cone2.id],
                    cone_positions=[(cone1.position.x, cone1.position.y), 
                                  (cone2.position.x, cone2.position.y)],
                    center=(center_x, center_y),
                    confidence=min(cone1.confidence, cone2.confidence)
                )
                areas.append(area)
                
                self.get_logger().info(f"🎯 열린 구역 발견: 거리={distance:.2f}m, 중심=({center_x:.2f}, {center_y:.2f})")
        
        return areas
    
    def publish_labeled_cone_map(self):
        """라벨링된 콘 맵 퍼블리시"""
        if not self.last_cone_map:
            return
        
        # 기존 콘 맵 복사
        labeled_map = ConeMapMsg()
        labeled_map.header = self.last_cone_map.header
        labeled_map.coordinate_system_ready = self.last_cone_map.coordinate_system_ready
        labeled_map.origin = self.last_cone_map.origin
        labeled_map.rotation_angle = self.last_cone_map.rotation_angle
        labeled_map.total_cone_count = self.last_cone_map.total_cone_count
        labeled_map.reliable_cone_count = self.last_cone_map.reliable_cone_count
        labeled_map.last_update = self.get_clock().now().to_msg()
        
        # 콘에 라벨 추가
        for cone in self.last_cone_map.cones:
            labeled_cone = ConeInfo()
            labeled_cone.id = cone.id
            labeled_cone.position = cone.position
            labeled_cone.original_position = cone.original_position
            labeled_cone.side_label = cone.side_label
            labeled_cone.confidence = cone.confidence
            labeled_cone.observation_count = cone.observation_count
            labeled_cone.last_seen = cone.last_seen
            
            # 구역 라벨 설정
            area_label = "unknown"
            for area in self.parking_areas:
                if cone.id in area.cone_ids:
                    area_label = f"{area.status.value}_area_{area.area_id}"
                    break
            
            labeled_cone.area_label = area_label
            labeled_map.cones.append(labeled_cone)
        
        self.labeled_cone_publisher.publish(labeled_map)
    
    def publish_open_slot(self):
        """열린 슬롯 위치 퍼블리시"""
        open_areas = [area for area in self.parking_areas if area.status == AreaStatus.OPEN]
        
        if not open_areas:
            return
        
        # 가장 신뢰도 높은 열린 구역 선택
        best_area = max(open_areas, key=lambda a: a.confidence)
        
        # 원본 좌표계로 변환
        if not self.last_cone_map:
            return
            
        # 정규화된 좌표 → 원본 좌표 변환
        normalized_x, normalized_y = best_area.center
        
        cos_angle = math.cos(self.last_cone_map.rotation_angle)
        sin_angle = math.sin(self.last_cone_map.rotation_angle)
        
        # 역회전 변환
        rotated_x = normalized_x * cos_angle - normalized_y * sin_angle
        rotated_y = normalized_x * sin_angle + normalized_y * cos_angle
        
        # 원점 오프셋 추가
        original_x = rotated_x + self.last_cone_map.origin.x
        original_y = rotated_y + self.last_cone_map.origin.y
        
        # PoseStamped 메시지 생성
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = original_x
        pose_msg.pose.position.y = original_y
        pose_msg.pose.position.z = 0.0
        
        # 좌표계 회전각도를 차량 방향으로 설정
        pose_msg.pose.orientation.w = math.cos(self.last_cone_map.rotation_angle / 2.0)
        pose_msg.pose.orientation.z = math.sin(self.last_cone_map.rotation_angle / 2.0)
        
        self.open_slot_publisher.publish(pose_msg)
        
        self.get_logger().info(f"🎯 열린 슬롯 퍼블리시: ({original_x:.2f}, {original_y:.2f}), 신뢰도={best_area.confidence:.1f}")
    
    def publish_visualization(self):
        """구역 판단 결과 시각화"""
        if not self.last_cone_map or not self.parking_areas:
            return
        
        marker_array = MarkerArray()
        
        for area in self.parking_areas:
            # 구역 중심점 마커
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'parking_areas'
            marker.id = area.area_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 정규화된 좌표 → 원본 좌표 변환
            cos_angle = math.cos(self.last_cone_map.rotation_angle)
            sin_angle = math.sin(self.last_cone_map.rotation_angle)
            
            rotated_x = area.center[0] * cos_angle - area.center[1] * sin_angle
            rotated_y = area.center[0] * sin_angle + area.center[1] * cos_angle
            
            marker.pose.position.x = rotated_x + self.last_cone_map.origin.x
            marker.pose.position.y = rotated_y + self.last_cone_map.origin.y
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            # 크기
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 1.0
            
            # 색상 (열린 구역: 녹색, 닫힌 구역: 빨간색)
            if area.status == AreaStatus.OPEN:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            marker.color.a = 0.6
            marker_array.markers.append(marker)
            
            # 텍스트 라벨
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'area_labels'
            text_marker.id = area.area_id + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = marker.pose
            text_marker.pose.position.z = 1.5
            
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"Area {area.area_id}\n{area.status.value.upper()}\nConf: {area.confidence:.1f}"
            
            marker_array.markers.append(text_marker)
        
        self.visualization_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AreaDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
