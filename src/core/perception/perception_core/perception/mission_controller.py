#!/usr/bin/env python3
"""
Lane-based Mission Controller for Perception System

이 노드는 localization에서 제공하는 현재 lane ID를 기반으로
각 미션별 perception 노드들의 활성화/비활성화를 제어합니다.

주요 기능:
1. 현재 lane ID 구독
2. lane ID에 따른 미션별 enable/disable 상태 결정
3. 각 perception 모듈에 enable 신호 발행
4. 다중 구간 및 단일 lane 미션 지원

미션별 lane ID 매핑 예시 (kcity 기준):
- 신호등 미션: lane 10-15, 45-47 (다중 교차로 구간)
- 표지판 미션: lane 20-25 (배달 구역) 
- 장애물 미션: lane 30-35, 38-40 (다중 장애물 회피 구간)
- 라바콘 미션: lane 50-60 (주차 구간)

지원하는 설정 형태:
- 단일 구간: {'start': 10, 'end': 15}
- 단일 lane: {'start': 25, 'end': 25}
- 다중 구간: [{'start': 10, 'end': 15}, {'start': 20, 'end': 25}]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class LaneMissionController(Node):
    """Lane ID 기반 미션 제어 노드"""
    
    def __init__(self):
        super().__init__('lane_mission_controller')
        
        # 미션별 enable 상태 초기화
        self.mission_states = {
            'trafficlight': False,
            'sign': False, 
            'obstacle': False,
            'rubber': False
        }
        
        # 현재 lane ID
        self.current_lane_id = 0
        
        # lane-mission 매핑 로드
        self.load_mission_config()
        
        # 구독자: localization에서 현재 lane ID 구독
        self.lane_id_subscriber = self.create_subscription(
            Int64,
            '/current_lanelet_id',  # localization 노드에서 발행하는 토픽
            self.lane_id_callback,
            10
        )
        
        # 발행자: 각 미션별 enable 토픽
        self.mission_publishers = {
            'trafficlight': self.create_publisher(Bool, '/mission/trafficlight/enable', 10),
            'sign': self.create_publisher(Bool, '/mission/sign/enable', 10),
            'obstacle': self.create_publisher(Bool, '/mission/obstacle/enable', 10),
            'rubber': self.create_publisher(Bool, '/mission/rubber/enable', 10)
        }

        # 타이머: 주기적으로 enable 상태 발행 (10Hz)
        self.timer = self.create_timer(1.0/10.0, self.publish_mission_states)

        self.get_logger().info('Lane Mission Controller 시작됨')
        self.get_logger().info(f'미션 매핑: {self.mission_mapping}')
    
    def load_mission_config(self):
        """미션 설정 파일 로드"""
        try:
            # config 파일 경로
            package_share_dir = get_package_share_directory('perception')
            config_path = os.path.join(package_share_dir, 'config', 'kcity_config.yaml')
            
            # YAML 파일이 없으면 기본값 사용
            if not os.path.exists(config_path):
                self.get_logger().warn(f'설정 파일을 찾을 수 없습니다: {config_path}')
                self.set_default_mission_mapping()
                return
                
            with open(config_path, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)
                self.mission_mapping = config.get('mission_mapping', {})
                
        except Exception as e:
            self.get_logger().error(f'설정 파일 로드 실패: {e}')
            self.set_default_mission_mapping()
    
    def set_default_mission_mapping(self):
        """기본 미션 매핑 설정 - 다중 구간 지원"""
        self.mission_mapping = {
            'trafficlight': [
                {'start': 3, 'end': 3},
                {'start': 5, 'end': 5},
                {'start': 7, 'end': 7},
                {'start': 9, 'end': 9},
                {'start': 13, 'end': 13},
                {'start': 15, 'end': 15},
                {'start': 17, 'end': 17}
            ],
            'sign': [
                {'start': 1, 'end': 1},   # 배달A
                {'start': 15, 'end': 15},   # 배달B
            ],
            'obstacle': [
                {'start': 7, 'end': 7},  # 대형 정적장애물
                {'start': 19, 'end': 19}   # 소형 정적장애물
            ],
            'rubber': [
                {'start': 21, 'end': 21}   # 주차
            ]
        }
        self.get_logger().info('본선 미션 매핑 사용')
    
    def lane_id_callback(self, msg):
        """현재 lane ID 수신 콜백"""
        self.current_lane_id = msg.data
        self.update_mission_states()
        
        self.get_logger().debug(f'현재 Lane ID: {self.current_lane_id}')
    
    def update_mission_states(self):
        """현재 lane ID에 따른 미션 상태 업데이트 - 다중 구간 지원"""
        for mission, lane_ranges in self.mission_mapping.items():
            # 단일 구간 형태도 지원하기 위한 호환성 처리
            if isinstance(lane_ranges, dict):
                lane_ranges = [lane_ranges]
            
            # 여러 구간 중 하나라도 현재 위치가 포함되면 활성화
            is_active = any(
                lane_range['start'] <= self.current_lane_id <= lane_range['end']
                for lane_range in lane_ranges
            )
            
            # 상태가 변경된 경우에만 로그 출력
            if self.mission_states[mission] != is_active:
                self.mission_states[mission] = is_active
                status = "활성화" if is_active else "비활성화"
                
                # 활성화된 구간 정보도 함께 로그에 출력
                if is_active:
                    active_ranges = [
                        f"[{r['start']}-{r['end']}]" 
                        for r in lane_ranges 
                        if r['start'] <= self.current_lane_id <= r['end']
                    ]
                    self.get_logger().info(
                        f'{mission} 미션 {status} (Lane {self.current_lane_id}, 구간: {", ".join(active_ranges)})'
                    )
                else:
                    self.get_logger().info(f'{mission} 미션 {status} (Lane {self.current_lane_id})')
    
    def publish_mission_states(self):
        """미션별 enable 상태 발행"""
        for mission, is_enabled in self.mission_states.items():
            msg = Bool()
            msg.data = is_enabled
            self.mission_publishers[mission].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LaneMissionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
