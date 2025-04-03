#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree

from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                           QVBoxLayout, QWidget, QGraphicsView, QGraphicsScene,
                           QLabel, QHBoxLayout)
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QPainterPath, QPen, QColor, QPainter

from autocar_nav.osm_handler import OSMHandler
from autocar_nav.yaw_to_quaternion import yaw_to_quaternion

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

def euclidean_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

class MapCanvas(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setRenderHint(QPainter.Antialiasing)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setRenderHint(QPainter.TextAntialiasing)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
        elif event.button() == Qt.RightButton and self.parent:
            self.parent.cancel_last_selection()
        elif self.parent and self.parent.is_selecting:
            self.parent.handle_mouse_click(event)
        super().mousePressEvent(event)
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.setDragMode(QGraphicsView.NoDrag)
        super().mouseReleaseEvent(event)
    
    def wheelEvent(self, event):
        # 줌 인/아웃
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1.0 / zoom_factor, 1.0 / zoom_factor)
        event.accept()

class PathSelector(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Path Selector')
        self.setGeometry(100, 100, 1200, 800)
        
        # ROS 노드 초기화
        self.node = Node('path_selector')
        
        # Publisher 설정
        self.path_pub = self.node.create_publisher(
            PoseArray,
            '/autocar/goals',
            10
        )
        
        # OSM 데이터 초기화
        self.osm_handler = None
        self.ways = {}
        self.way_nodes = {}
        self.selected_path = []
        
        # GUI 요소 초기화
        self.init_ui()
        
        # ROS 스핀을 위한 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(100)  # 100ms 간격으로 ROS 스핀
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # 버튼 생성
        self.load_map_btn = QPushButton('지도 로드')
        self.load_map_btn.clicked.connect(self.load_map)
        
        self.select_path_btn = QPushButton('경로 선택')
        self.select_path_btn.clicked.connect(self.select_path)
        
        self.publish_path_btn = QPushButton('경로 발행')
        self.publish_path_btn.clicked.connect(self.publish_path)
        
        self.clear_path_btn = QPushButton('경로 초기화')
        self.clear_path_btn.clicked.connect(self.clear_path)
        
        # 정보 표시 레이블
        info_layout = QHBoxLayout()
        self.loaded_ways_label = QLabel('로드된 경로 수: 0')
        self.selected_ways_label = QLabel('선택된 경로 ID: ')
        info_layout.addWidget(self.loaded_ways_label)
        info_layout.addWidget(self.selected_ways_label)
        
        # 레이아웃에 위젯 추가
        layout.addLayout(info_layout)
        layout.addWidget(self.load_map_btn)
        layout.addWidget(self.select_path_btn)
        layout.addWidget(self.publish_path_btn)
        layout.addWidget(self.clear_path_btn)
        
        # 지도 표시를 위한 캔버스
        self.map_canvas = MapCanvas(self)
        self.map_scene = QGraphicsScene()
        self.map_canvas.setScene(self.map_scene)
        layout.addWidget(self.map_canvas)
        
        # 선택 모드 상태 변수 추가
        self.is_selecting = False
        
    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        
    def closeEvent(self, event):
        self.timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()
        
    def load_map(self):
        try:
            # OSM 파일 경로 설정
            osm_file_path = os.path.join(get_package_share_directory('autocar_nav'), 'data')
            # osm_files = ["hitech2_LINK.osm", "hitech2_INTERSECTION_LINK.osm", "hitech2_STOPLINE.osm"]
            osm_files = ["KCITY_MAIN.osm", "KCITY_INTERSECTION_LINK_MAIN.osm"]
            
            # OSM 핸들러 초기화
            self.osm_handler = OSMHandler()
            
            # 각 파일 로드
            for osm_file in osm_files:
                file_path = os.path.join(osm_file_path, osm_file)
                self.node.get_logger().info(f'파일 로드 중: {osm_file}')
                self.osm_handler.import_file(file_path)
            
            # 데이터 저장
            self.ways = self.osm_handler.ways
            self.way_nodes = self.osm_handler.way_nodes
            
            # 지도 그리기
            self.draw_map()
            
            # 로드된 경로 수 업데이트
            self.loaded_ways_label.setText(f'로드된 경로 수: {len(self.ways)}')
            
            self.node.get_logger().info('모든 지도 파일 로드 완료')
            
        except Exception as e:
            self.node.get_logger().error(f'지도 로드 실패: {str(e)}')
    
    def draw_map(self):
        # 기존 씬 초기화
        self.map_scene.clear()
        
        # 도로 네트워크 그리기
        road_pen = QPen(QColor(100, 100, 100))
        road_pen.setWidth(2)
        
        for way_id, nodes in self.ways.items():
            path = QPainterPath()
            for i, node_id in enumerate(nodes):
                pos = self.way_nodes[node_id]
                if i == 0:
                    path.moveTo(pos[0], pos[1])
                else:
                    path.lineTo(pos[0], pos[1])
            
            self.map_scene.addPath(path, road_pen)
        
        # 선택된 경로 하이라이트
        self.highlight_selected_path()
        
        # 씬 크기 조정
        self.map_canvas.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
    
    def select_path(self):
        self.is_selecting = not self.is_selecting
        if self.is_selecting:
            self.node.get_logger().info('경로 선택 모드 활성화')
            self.select_path_btn.setStyleSheet("background-color: #90EE90;")
        else:
            self.node.get_logger().info('경로 선택 모드 비활성화')
            self.select_path_btn.setStyleSheet("")
    
    def handle_mouse_click(self, event):
        if not self.is_selecting:
            return
            
        # 클릭한 위치의 좌표 변환
        scene_pos = self.map_canvas.mapToScene(event.pos())
        
        # 가장 가까운 way 찾기
        closest_way = self.find_closest_way(scene_pos)
        
        if closest_way and closest_way not in self.selected_path:
            self.selected_path.append(closest_way)
            self.highlight_selected_path()
            self.node.get_logger().info(f'경로 추가: {closest_way}')
            
            # 선택된 경로 ID 업데이트
            self.selected_ways_label.setText(f'선택된 경로 ID: {", ".join(map(str, self.selected_path))}')
            
            # 선택된 경로의 노드 수 표시
            if closest_way in self.ways:
                node_count = len(self.ways[closest_way])
                self.node.get_logger().info(f'선택된 경로의 노드 수: {node_count}')
    
    def find_closest_way(self, point):
        min_dist = float('inf')
        closest_way = None
        click_pos = (point.x(), point.y())
        
        for way_id, nodes in self.ways.items():
            for i in range(len(nodes)-1):
                # 현재 노드와 다음 노드 사이의 선분과의 거리 계산
                current_pos = self.way_nodes[nodes[i]]
                next_pos = self.way_nodes[nodes[i+1]]
                
                # 선분과 점 사이의 거리 계산
                dist = self.point_to_line_distance(click_pos, current_pos, next_pos)
                if dist < min_dist:
                    min_dist = dist
                    closest_way = way_id
        
        # 선택 거리 임계값을 20으로 증가
        return closest_way if min_dist < 20 else None
    
    def point_to_line_distance(self, point, line_start, line_end):
        # 점과 선분 사이의 최단 거리 계산
        x, y = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 선분의 길이
        line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if line_length == 0:
            return np.sqrt((x - x1)**2 + (y - y1)**2)
        
        # 점에서 선분으로의 투영 비율 계산
        t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / (line_length**2)
        
        if t < 0:
            return np.sqrt((x - x1)**2 + (y - y1)**2)
        elif t > 1:
            return np.sqrt((x - x2)**2 + (y - y2)**2)
        
        # 선분 위의 투영점 계산
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        return np.sqrt((x - proj_x)**2 + (y - proj_y)**2)
    
    def highlight_selected_path(self):
        # 선택된 경로 하이라이트
        highlight_pen = QPen(QColor(255, 0, 0))
        highlight_pen.setWidth(3)
        
        for way_id in self.selected_path:
            if way_id in self.ways:
                path = QPainterPath()
                nodes = self.ways[way_id]
                for i, node_id in enumerate(nodes):
                    pos = self.way_nodes[node_id]
                    if i == 0:
                        path.moveTo(pos[0], pos[1])
                    else:
                        path.lineTo(pos[0], pos[1])
                self.map_scene.addPath(path, highlight_pen)
                
                # 시작점과 끝점 표시
                start_pos = self.way_nodes[nodes[0]]
                end_pos = self.way_nodes[nodes[-1]]
                self.map_scene.addEllipse(start_pos[0]-2, start_pos[1]-2, 4, 4, QPen(QColor(0, 255, 0)))
                self.map_scene.addEllipse(end_pos[0]-2, end_pos[1]-2, 4, 4, QPen(QColor(255, 0, 0)))
    
    def publish_path(self):
        if not self.selected_path:
            self.node.get_logger().warn('선택된 경로가 없습니다')
            return
        
        try:
            # 선택된 경로를 PoseArray 메시지로 변환
            path_msg = PoseArray()
            path_msg.header.frame_id = "world"
            path_msg.header.stamp = self.node.get_clock().now().to_msg()
            
            for way_id in self.selected_path:
                if way_id in self.ways:
                    nodes = self.ways[way_id]
                    for i in range(len(nodes)-1):
                        current_pos = self.way_nodes[nodes[i]]
                        next_pos = self.way_nodes[nodes[i+1]]
                        
                        pose = Pose()
                        pose.position.x = current_pos[0]
                        pose.position.y = current_pos[1]
                        
                        # 방향 계산
                        yaw = np.arctan2(next_pos[1] - current_pos[1],
                                       next_pos[0] - current_pos[0])
                        quaternion = yaw_to_quaternion(yaw)
                        pose.orientation.x = quaternion.x
                        pose.orientation.y = quaternion.y
                        pose.orientation.z = quaternion.z
                        pose.orientation.w = quaternion.w
                        
                        path_msg.poses.append(pose)
            
            # 마지막 포인트 추가
            last_way_id = self.selected_path[-1]
            if last_way_id in self.ways:
                last_node = self.ways[last_way_id][-1]
                last_pos = self.way_nodes[last_node]
                last_pose = Pose()
                last_pose.position.x = last_pos[0]
                last_pose.position.y = last_pos[1]
                path_msg.poses.append(last_pose)
            
            # 경로 발행
            self.path_pub.publish(path_msg)
            self.node.get_logger().info('경로 발행 완료')
            
        except Exception as e:
            self.node.get_logger().error(f'경로 발행 실패: {str(e)}')
    
    def clear_path(self):
        self.selected_path = []
        self.draw_map()
        self.selected_ways_label.setText('선택된 경로 ID: ')
        self.node.get_logger().info('경로 초기화 완료')

    def cancel_last_selection(self):
        if self.selected_path:
            removed_way = self.selected_path.pop()
            self.node.get_logger().info(f'마지막 선택 경로 취소: {removed_way}')
            self.selected_ways_label.setText(f'선택된 경로 ID: {", ".join(map(str, self.selected_path))}')
            self.draw_map()  # 지도 다시 그리기

def main():
    # ROS2 초기화
    rclpy.init()
    
    # QApplication 생성
    app = QApplication(sys.argv)
    
    # GUI 생성 및 표시
    gui = PathSelector()
    gui.show()
    
    # 이벤트 루프 실행
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 