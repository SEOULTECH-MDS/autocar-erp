#!/usr/bin/env python3
"""
Mock Localization Node for Testing Lane Mission Controller

실제 localization 노드가 없을 때 테스트용으로 사용하는 모의 노드입니다.
주기적으로 lane ID를 변경하여 미션 전환을 시뮬레이션합니다.

사용법:
1. 실제 localization 노드 대신 실행
2. lane ID가 순환하면서 각 미션이 활성화/비활성화되는 것을 확인
3. RViz나 로그를 통해 동작 확인 가능
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time


class MockLocalizationNode(Node):
    """테스트용 모의 Localization 노드"""
    
    def __init__(self):
        super().__init__('mock_localization')
        
        # 현재 lane ID
        self.current_lane_id = 0
        
        # 테스트용 lane ID 시퀀스 (각 미션 구간을 순환)
        self.lane_sequence = [
            5,   # 미션 외 구간
            12,  # 신호등 미션 (10-15)
            13,  # 신호등 미션
            18,  # 미션 외 구간  
            22,  # 표지판 미션 (20-25)
            23,  # 표지판 미션
            28,  # 미션 외 구간
            35,  # 장애물 미션 (30-40)
            38,  # 장애물 미션
            45,  # 미션 외 구간
            55,  # 라바콘 미션 (50-60)
            58,  # 라바콘 미션
            65,  # 미션 외 구간
        ]
        
        self.sequence_index = 0
        
        # Lane ID 발행자
        self.lane_id_publisher = self.create_publisher(
            Int32,
            '/localization/current_lane_id',
            10
        )
        
        # 타이머: 3초마다 lane ID 변경
        self.timer = self.create_timer(3.0, self.update_lane_id)
        
        self.get_logger().info('Mock Localization 노드 시작')
        self.get_logger().info(f'Lane 시퀀스: {self.lane_sequence}')
    
    def update_lane_id(self):
        """Lane ID 업데이트 및 발행"""
        # 시퀀스에서 다음 lane ID 가져오기
        self.current_lane_id = self.lane_sequence[self.sequence_index]
        
        # Lane ID 발행
        msg = Int32()
        msg.data = self.current_lane_id
        self.lane_id_publisher.publish(msg)
        
        # 미션 구간 판단
        mission_info = self.get_mission_info(self.current_lane_id)
        
        self.get_logger().info(
            f'Lane ID: {self.current_lane_id} -> {mission_info}'
        )
        
        # 다음 인덱스로 이동 (순환)
        self.sequence_index = (self.sequence_index + 1) % len(self.lane_sequence)
    
    def get_mission_info(self, lane_id):
        """Lane ID에 해당하는 미션 정보 반환"""
        if 10 <= lane_id <= 15:
            return "신호등 미션 구간"
        elif 20 <= lane_id <= 25:
            return "표지판 미션 구간"
        elif 30 <= lane_id <= 40:
            return "장애물 미션 구간"
        elif 50 <= lane_id <= 60:
            return "라바콘 미션 구간"
        else:
            return "미션 외 구간"


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MockLocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
