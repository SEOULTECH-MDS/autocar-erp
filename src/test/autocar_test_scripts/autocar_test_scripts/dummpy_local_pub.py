#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import os
from .osm_handler import OSMHandler
from autocar_utils.yaw_to_quaternion import yaw_to_quaternion


class DummyLocalPub(Node):
    def __init__(self):
        super().__init__('dummy_local_pub')
        
        # 퍼블리셔 설정
        self.local_path_pub = self.create_publisher(Path, '/waypoints', 10)
        
        # OSM 파일 경로 설정
        self.osm_file_path = os.path.join(
            os.path.dirname(__file__), 
            'dummy_parking.osm'
        )
        
        # OSM Handler 초기화
        self.osm_handler = OSMHandler()
        
        # 파싱된 waypoints와 path 메시지를 저장할 변수
        self.waypoints = []
        self.path_msg = None
        self.waypoints_loaded = False
        
        self.get_logger().info("DummyLocalPub 노드가 시작되었습니다.")
        
        # OSM 파일 파싱 (최초 한 번만)
        self.load_osm_waypoints()
        
        # 타이머 설정 - 1초마다 한 번 발행
        self.create_timer(1.0, self.publish_local_waypoints)

    def load_osm_waypoints(self):
        """OSM 파일에서 waypoints를 로드 (최초 한 번만 실행)"""
        if self.waypoints_loaded:
            return
        
        if not os.path.exists(self.osm_file_path):
            self.get_logger().error(f"OSM 파일을 찾을 수 없습니다: {self.osm_file_path}")
            return
        
        try:
            # OSM 파일 파싱
            self.osm_handler.import_file(self.osm_file_path)
            
            # 주행유도선 waypoints 추출 (code가 0 또는 1인 경우)
            self.waypoints = []
            
            # OSM Handler에서 ways 정보를 순서대로 추출
            for way_id in sorted(self.osm_handler.ways.keys()):
                way_type = self.osm_handler.ways_info['type'][way_id]
                
                # 주행유도선 (code 0 또는 1)만 처리
                if way_type == 0 or way_type == 1:
                    node_ids = self.osm_handler.ways[way_id]
                    
                    for node_id in node_ids:
                        if node_id in self.osm_handler.way_nodes:
                            x, y = self.osm_handler.way_nodes[node_id]
                            self.waypoints.append({'x': x, 'y': y})
            
            if self.waypoints:
                # Path 메시지 생성 (한 번만)
                self.path_msg = self.create_path_message(self.waypoints)
                self.waypoints_loaded = True
                self.get_logger().info(f"OSM waypoints 로딩 완료: {len(self.waypoints)}개 포인트")
            else:
                self.get_logger().error("OSM 파일에서 주행유도선 waypoints를 찾을 수 없습니다.")
                
        except Exception as e:
            self.get_logger().error(f"OSM 파일 파싱 중 오류 발생: {e}")

    def create_path_message(self, waypoints):
        """waypoints로부터 Path 메시지 생성"""
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 방향 계산을 미리 수행
        num_waypoints = len(waypoints)
        
        for i, wp in enumerate(waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.header.stamp = path_msg.header.stamp  # 같은 타임스탬프 재사용
            
            # 위치 설정
            pose_stamped.pose.position.x = wp['x']
            pose_stamped.pose.position.y = wp['y']
            pose_stamped.pose.position.z = 0.0
            
            # 방향 설정 최적화
            if i < num_waypoints - 1:
                next_wp = waypoints[i + 1]
                yaw = math.atan2(next_wp['y'] - wp['y'], next_wp['x'] - wp['x'])
            elif i > 0:
                prev_wp = waypoints[i - 1]
                yaw = math.atan2(wp['y'] - prev_wp['y'], wp['x'] - prev_wp['x'])
            else:
                yaw = 0.0
            
            q = yaw_to_quaternion(yaw)
            pose_stamped.pose.orientation.x = q.x
            pose_stamped.pose.orientation.y = q.y
            pose_stamped.pose.orientation.z = q.z
            pose_stamped.pose.orientation.w = q.w
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg

    def publish_local_waypoints(self):
        """로컬 waypoints를 주기적으로 발행"""
        if not self.waypoints_loaded or self.path_msg is None:
            self.get_logger().warn("OSM waypoints가 아직 로드되지 않았습니다.")
            return
        
        # 헤더 타임스탬프만 업데이트 (효율성을 위해)
        current_time = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = current_time
        
        # Path 메시지 발행
        self.local_path_pub.publish(self.path_msg)
        self.get_logger().info(f"OSM waypoints 발행: {len(self.path_msg.poses)}개 포인트")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        dummy_local_pub = DummyLocalPub()
        rclpy.spin(dummy_local_pub)
    except KeyboardInterrupt:
        pass
    finally:
        if 'dummy_local_pub' in locals():
            dummy_local_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
