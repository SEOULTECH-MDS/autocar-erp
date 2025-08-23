#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from planning_msgs.msg import ObstacleArray, Obstacle
import numpy as np


class ConesNode(Node):
    """
    22개의 라바콘을 지정된 위치에 배치하고 퍼블리싱하는 노드
    """
    
    def __init__(self):
        super().__init__('cones_node')
        
        # ──── 패턴 선택 파라미터 ───────────────────────────────────────────────
        self.declare_parameter('open_area_pattern', 1)  # 0, 1, 2 중 선택 (기본값: Area 2 열림)
        # ──── 순차 인식 모사 파라미터 ─────────────────────────────────────────
        self.declare_parameter('sequential_mode', True)   # True면 순차 공개
        self.declare_parameter('seq_publish_rate', 0.5)   # Hz
        self.declare_parameter('seq_order', 'near_to_far') # near_to_far | far_to_near | random
        self.declare_parameter('seq_dropout_prob', 0.0)    # 0.0~0.5 일부 프레임 드롭
        self.declare_parameter('seq_noise_std', 0.1)       # m, 위치 잡음
        self.declare_parameter('seq_step_per_tick', 3)     # 한 틱에 공개할 콘 개수
        
        # 퍼블리셔 초기화
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/cones_markers', 
            10
        )
        
        # 플래너용 장애물 퍼블리셔 추가
        self.obstacle_pub = self.create_publisher(
            ObstacleArray,
            '/cones_obstacles',
            10
        )
        
        # 타이머 설정
        rate = float(self.get_parameter('seq_publish_rate').value)
        self.timer = self.create_timer(1.0 / max(1e-3, rate), self.publish_cones)
        
        # 라바콘 마커 설정
        self.cone_markers = MarkerArray()
        
        # 라바콘의 기본 위치 경우의 수 3가지 (상대 좌표)
        self.relative_positions_Area_2_Open = [
            (0, 0), (0, 1.25), (0, 2.5), (0, 3.75), (0, 5), (0, 6.25), (0, 7.5), (0, 8.75), (0, 10), (0, 15),
            (1.25, 0), (1.25, 5), (1.25, 10), (1.25, 15),
            (2.5, 0), (2.5, 2.5), (2.5, 5), (2.5, 7.5), (2.5, 10), (2.5, 12.5), (2.5, 15)
        ]

        self.relative_positions_Area_1_Open = [
            (0, 0), (0, 1.25), (0, 2.5), (0, 3.75), (0, 5), (0, 10), (0, 11.25), (0, 12.5), (0, 13.75), (0, 15),
            (1.25, 0), (1.25, 5), (1.25, 10), (1.25, 15),
            (2.5, 0), (2.5, 2.5), (2.5, 5), (2.5, 7.5), (2.5, 10), (2.5, 12.5), (2.5, 15)
        ]

        self.relative_positions_Area_0_Open = [
            (0, 0), (0, 5), (0, 6.25), (0, 7.5), (0, 8.75), (0, 10), (0, 11.25), (0, 12.5), (0, 13.75), (0, 15),
            (1.25, 0), (1.25, 5), (1.25, 10), (1.25, 15),
            (2.5, 0), (2.5, 2.5), (2.5, 5), (2.5, 7.5), (2.5, 10), (2.5, 12.5), (2.5, 15)
        ]

        # ──── 패턴 선택 로직 ───────────────────────────────────────────────
        # 파라미터 로드
        open_area_id = self.get_parameter('open_area_pattern').value
        
        # 선택된 패턴에 따라 사용할 콘 위치 결정
        if open_area_id == 0:
            self.relative_positions = self.relative_positions_Area_0_Open
            pattern_name = "Area 0 Open (아래쪽 열림)"
        elif open_area_id == 1:
            self.relative_positions = self.relative_positions_Area_1_Open  
            pattern_name = "Area 1 Open (중간 열림)"
        else:  # default: 2
            self.relative_positions = self.relative_positions_Area_2_Open
            pattern_name = "Area 2 Open (위쪽 열림)"
        
        # 전체 라바콘 그룹의 기준점 (0,0) 위치 (절대 좌표)
        self.group_reference_x = 3.0
        self.group_reference_y = 7.5
        self.group_reference_z = 0.0  # 2D 충돌을 위해 0.0으로 설정
        
        # 라바콘의 기본 크기
        self.cone_diameter = 0.3
        self.cone_height = 0.5
        self.cone_radius = 0.15  # 충돌 검사용 반지름
        
        # 라바콘의 기본 색상
        self.cone_color = (1.0, 0.5, 0.0, 0.8)  # 주황색
        
        # 순차 인식 모사 인덱스 준비(마커/장애물 초기화 전에 준비)
        self._seq_indices = list(range(len(self.relative_positions)))
        order = str(self.get_parameter('seq_order').value)
        if order == 'near_to_far':
            # y 기준 오름차순
            self._seq_indices.sort(key=lambda i: self.relative_positions[i][1])
        elif order == 'far_to_near':
            self._seq_indices.sort(key=lambda i: self.relative_positions[i][1], reverse=True)
        elif order == 'random':
            import random
            random.shuffle(self._seq_indices)
        self._seq_cursor = 0

        # 라바콘 마커 초기화
        self.setup_cone_markers()
        
        self.get_logger().info(f'Cones node가 시작되었습니다. {len(self.relative_positions)}개의 라바콘이 배치되었습니다.')
        self.get_logger().info(f'선택된 패턴: {pattern_name} (open_area_pattern={open_area_id})')
    
    def setup_cone_markers(self):
        """
        22개의 라바콘 마커를 설정합니다.
        """
        self.cone_markers.markers.clear()
        
        for i, (rel_x, rel_y) in enumerate(self.relative_positions):
            cone_marker = Marker()
            cone_marker.header.frame_id = "map"
            cone_marker.header.stamp = self.get_clock().now().to_msg()
            cone_marker.ns = "cones"
            cone_marker.id = i
            cone_marker.type = Marker.CYLINDER
            cone_marker.action = Marker.ADD
            
            # 절대 위치 계산 (그룹 기준점 + 상대 위치)
            abs_x = self.group_reference_x + rel_x
            abs_y = self.group_reference_y + rel_y
            abs_z = self.group_reference_z
            
            # 위치 설정 (2D 충돌을 위해 Z는 0.0으로 설정)
            cone_marker.pose.position.x = abs_x
            cone_marker.pose.position.y = abs_y
            cone_marker.pose.position.z = 0.0  # 2D 충돌을 위해 0.0으로 설정
            cone_marker.pose.orientation.w = 1.0
            
            # 크기 설정
            cone_marker.scale = Vector3()
            cone_marker.scale.x = self.cone_diameter
            cone_marker.scale.y = self.cone_diameter
            cone_marker.scale.z = self.cone_height
            
            # 색상 설정
            cone_marker.color = ColorRGBA()
            cone_marker.color.r = self.cone_color[0]
            cone_marker.color.g = self.cone_color[1]
            cone_marker.color.b = self.cone_color[2]
            cone_marker.color.a = self.cone_color[3]
            
            self.cone_markers.markers.append(cone_marker)
        
        # 장애물 배열 초기화
        self.obstacle_array = ObstacleArray()
        self.update_obstacle_array()
    
    def move_entire_group(self, reference_x, reference_y, reference_z=None):
        """
        전체 라바콘 그룹을 새로운 기준점 (0,0) 위치로 이동합니다.
        
        Args:
            reference_x (float): 새로운 그룹 기준점 X 좌표
            reference_y (float): 새로운 그룹 기준점 Y 좌표
            reference_z (float): 새로운 그룹 기준점 Z 좌표 (선택사항)
        """
        self.group_reference_x = reference_x
        self.group_reference_y = reference_y
        if reference_z is not None:
            self.group_reference_z = reference_z
        
        # 모든 라바콘의 위치 업데이트
        for i, (rel_x, rel_y) in enumerate(self.relative_positions):
            abs_x = self.group_reference_x + rel_x
            abs_y = self.group_reference_y + rel_y
            abs_z = self.group_reference_z
            
            self.cone_markers.markers[i].pose.position.x = abs_x
            self.cone_markers.markers[i].pose.position.y = abs_y
            self.cone_markers.markers[i].pose.position.z = 0.0  # 2D 충돌을 위해 0.0으로 설정
        
        # 장애물 배열 업데이트
        self.update_obstacle_array()
    
    def update_all_cones_size(self, diameter, height):
        """
        모든 라바콘의 크기를 일률적으로 변경합니다.
        
        Args:
            diameter (float): 라바콘의 지름
            height (float): 라바콘의 높이
        """
        self.cone_diameter = diameter
        self.cone_height = height
        
        self.cone_radius = diameter / 2.0  # 반지름 업데이트
        
        for marker in self.cone_markers.markers:
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = height
        
        # 장애물 배열 업데이트
        self.update_obstacle_array()
    
    def update_all_cones_color(self, r, g, b, a=0.8):
        """
        모든 라바콘의 색상을 일률적으로 변경합니다.
        
        Args:
            r (float): 빨간색 값 (0.0 ~ 1.0)
            g (float): 초록색 값 (0.0 ~ 1.0)
            b (float): 파란색 값 (0.0 ~ 1.0)
            a (float): 투명도 (0.0 ~ 1.0)
        """
        self.cone_color = (r, g, b, a)
        
        for marker in self.cone_markers.markers:
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
    
    def set_cone_color_by_type(self, color_type):
        """
        미리 정의된 색상 타입으로 모든 라바콘의 색상을 설정합니다.
        
        Args:
            color_type (str): 색상 타입 ('orange', 'red', 'blue', 'green', 'yellow')
        """
        color_map = {
            'orange': (1.0, 0.5, 0.0, 0.8),
            'red': (1.0, 0.0, 0.0, 0.8),
            'blue': (0.0, 0.0, 1.0, 0.8),
            'green': (0.0, 1.0, 0.0, 0.8),
            'yellow': (1.0, 1.0, 0.0, 0.8)
        }
        
        if color_type in color_map:
            self.update_all_cones_color(*color_map[color_type])
        else:
            self.get_logger().warn(f'알 수 없는 색상 타입: {color_type}')
    
    def get_group_reference(self):
        """
        현재 그룹의 기준점 (0,0) 위치를 반환합니다.
        
        Returns:
            tuple: (x, y, z) 기준점 좌표
        """
        return (self.group_reference_x, self.group_reference_y, self.group_reference_z)
    
    def update_obstacle_array(self):
        """
        현재 라바콘 위치를 기반으로 장애물 배열을 업데이트합니다.
        """
        self.obstacle_array.obstacles.clear()
        self.obstacle_array.header.frame_id = "map"
        self.obstacle_array.header.stamp = self.get_clock().now().to_msg()
        # 순차 모드면 일부만 공개
        sequential = bool(self.get_parameter('sequential_mode').value)
        dropout = float(self.get_parameter('seq_dropout_prob').value)
        noise_std = float(self.get_parameter('seq_noise_std').value)
        import random

        def maybe_noise(v: float) -> float:
            if noise_std <= 0.0:
                return v
            return float(v + np.random.normal(0.0, noise_std))

        indices = range(len(self.relative_positions)) if not sequential else self._seq_indices[: self._seq_cursor]
        for i in indices:
            rel_x, rel_y = self.relative_positions[i]
            if sequential and dropout > 0.0 and random.random() < dropout:
                continue
            obstacle = Obstacle()
            obstacle.id = i
            obstacle.type = "cone"
            abs_x = self.group_reference_x + rel_x
            abs_y = self.group_reference_y + rel_y
            obstacle.center.x = maybe_noise(abs_x)
            obstacle.center.y = maybe_noise(abs_y)
            obstacle.center.z = 0.0
            obstacle.radius = self.cone_radius
            self.obstacle_array.obstacles.append(obstacle)
    
    def publish_cones(self):
        """
        라바콘 마커를 퍼블리시합니다.
        """
        # 타임스탬프 업데이트
        for marker in self.cone_markers.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        
        # 마커 퍼블리시
        self.marker_pub.publish(self.cone_markers)
        
        # 장애물 배열 퍼블리시
        # 순차 모드에서는 커서를 전진시키며 업데이트
        if bool(self.get_parameter('sequential_mode').value):
            if self._seq_cursor < len(self._seq_indices):
                step = int(self.get_parameter('seq_step_per_tick').value)
                if step <= 0:
                    step = 1
                self._seq_cursor = min(len(self._seq_indices), self._seq_cursor + step)
            # 동기화된 마커 업데이트를 위해 마커 위치는 고정(시각화는 전체), 장애물만 점진 공개
            self.update_obstacle_array()
        else:
            self.update_obstacle_array()
        self.obstacle_pub.publish(self.obstacle_array)


def main(args=None):
    rclpy.init(args=args)
    
    cones_node = ConesNode()
    
    try:
        rclpy.spin(cones_node)
    except KeyboardInterrupt:
        pass
    finally:
        cones_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
