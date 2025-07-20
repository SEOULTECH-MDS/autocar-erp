#!/usr/bin/env python3
"""
TF Broadcaster - 좌표계 변환 브로드캐스터

이 파일은 RViz에서 3D 시각화를 위해 필요한 좌표계 변환(TF)을 브로드캐스트합니다.

주요 기능:
- map 좌표계에서 base_link 좌표계로의 변환 제공
- RViz에서 마커들이 올바른 위치에 표시되도록 함
- 정적 변환으로 차량의 기본 위치 설정

브로드캐스트하는 TF:
- parent_frame: "map"
- child_frame: "base_link"
- 변환: 원점 (0, 0, 0)에서 약간 위로 (0, 0, 0.5)

사용법:
    ros2 run path_planning tf_broadcaster

참고:
    이 노드는 RViz 시각화를 위해 필수적입니다.
    TF가 없으면 RViz에서 "No tf data" 오류가 발생합니다.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # TF 브로드캐스터 생성
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 타이머 설정 (100Hz로 TF 발행)
        self.timer = self.create_timer(0.01, self.broadcast_tf)
        
        self.get_logger().info('🗺️ TF Broadcaster 시작됨')
        
    def broadcast_tf(self):
        """map 프레임을 브로드캐스트"""
        t = TransformStamped()
        
        # 현재 시간 설정
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # 변환 정보 (차량을 원점에 위치)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # 회전 정보 (쿼터니언)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # TF 발행
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TFBroadcaster()
    
    try:
        rclpy.spin(tf_broadcaster)
    except KeyboardInterrupt:
        tf_broadcaster.get_logger().info('🗺️ TF Broadcaster 종료됨')
    finally:
        tf_broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 