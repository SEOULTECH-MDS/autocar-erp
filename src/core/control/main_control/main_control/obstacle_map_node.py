#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleMap(Node):
    def __init__(self):
        super().__init__('obstacle_map_node')

        # 파라미터 설정
        self.declare_parameter('velodyne_x_offset', 1.3)  # velodyne -> base_link: x축 오프셋
        self.declare_parameter('merge_radius', 0.2)        # 미터: 이 반경 내에서 병합
        self.declare_parameter('smoothing_alpha', 0.2)    # 매칭된 장애물의 EMA 평활화 계수
        self.declare_parameter('max_age_sec', 5.0)         # 이 시간동안 관측되지 않으면 제거
        self.declare_parameter('marker_scale', 0.3)        # RViz 포인트 크기

        self.velodyne_x_offset = float(self.get_parameter('velodyne_x_offset').value)
        self.merge_radius = float(self.get_parameter('merge_radius').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.max_age_sec = float(self.get_parameter('max_age_sec').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)

        # TF 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 추적된 장애물 저장소 (map 좌표계 기준)
        # 각 항목: {'x': float, 'y': float, 'last_seen': rclpy.time.Time}
        self.tracked = []

        # 구독자/퍼블리셔 설정
        self.obstacle_sub = self.create_subscription(MarkerArray, '/sensor_fusion/obstacles', self.obstacle_cb, 10)
        self.obstacle_map_pub = self.create_publisher(MarkerArray, '/obstacle_map', 10)

        # 주기적 퍼블리시 타이머 (새 데이터가 없어도 캐시된 장애물 재퍼블리시)
        self.timer_pub = self.create_timer(0.1, self.publish_obstacles)

    def obstacle_cb(self, msg: MarkerArray):
        now = self.get_clock().now()
        points_map = []

        # 0번째 마커만 사용
        if not msg.markers:
            self.remove_old(now)
            self.publish_obstacles()
            return

        mk = msg.markers[0]
        stamp = mk.header.stamp if mk.header.stamp.sec != 0 or mk.header.stamp.nanosec != 0 else now.to_msg()

        for p in mk.points:
            x_base = float(p.x) + self.velodyne_x_offset
            y_base = float(p.y)
            z_base = float(p.z)

            pm = self._transform_base_to_map(x_base, y_base, z_base, stamp)
            if pm is not None:
                points_map.append(pm)

        if not points_map:
            self.remove_old(now)
            self.publish_obstacles()
            return

        # 새로운 포인트로 추적된 장애물 업데이트 (반경 내 병합, EMA 평활화)
        for x_map, y_map, _ in points_map:
            idx = self._find_nearest(x_map, y_map, self.merge_radius)
            if idx is None:
                self.tracked.append({'x': x_map, 'y': y_map, 'last_seen': now})
            else:
                ox = self.tracked[idx]['x']
                oy = self.tracked[idx]['y']
                a = self.smoothing_alpha
                self.tracked[idx]['x'] = (1.0 - a) * ox + a * x_map
                self.tracked[idx]['y'] = (1.0 - a) * oy + a * y_map
                self.tracked[idx]['last_seen'] = now

        self.remove_old(now)
        self.publish_obstacles()

    def _transform_base_to_map(self, x_base: float, y_base: float, z_base: float, stamp):
        # base_link 좌표계의 PointStamped 생성
        ps = PointStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = stamp
        ps.point.x = x_base
        ps.point.y = y_base
        ps.point.z = z_base

        try:
            # 변환 가능 여부 확인 (timeout 증가)
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time.from_msg(stamp), timeout=Duration(seconds=0.5)):
                return None
            pm = self.tf_buffer.transform(ps, 'map', timeout=Duration(seconds=0.5))
            return (pm.point.x, pm.point.y, pm.point.z)
        except Exception as e:
            self.get_logger().debug(f'tf 변환 실패: {e}')
            return None

    def _find_nearest(self, x: float, y: float, radius: float):
        # 주어진 반경 내에서 가장 가까운 기존 장애물 찾기
        best_idx = None
        best_d2 = radius * radius  # 제곱 거리로 비교 (성능 최적화)
        for i, o in enumerate(self.tracked):
            dx = x - o['x']
            dy = y - o['y']
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best_d2 = d2
                best_idx = i
        return best_idx

    def remove_old(self, now):
        # 오래된 장애물 제거 (max_age_sec 초과)
        kept = []
        for o in self.tracked:
            age = (now - o['last_seen']).nanoseconds * 1e-9  # 나노초를 초로 변환
            if age <= self.max_age_sec:
                kept.append(o)
        if len(kept) != len(self.tracked):
            self.tracked = kept

    def publish_obstacles(self):
        # map 좌표계의 CYLINDER 마커들로 MarkerArray 생성
        ma = MarkerArray()
        
        # 각 장애물을 개별 CYLINDER 마커로 생성
        for i, o in enumerate(self.tracked):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacle_map'
            m.id = i  # 각 원통마다 고유 ID
            m.type = Marker.CYLINDER  # 원통 타입
            m.action = Marker.ADD
            
            # 원통 위치 설정
            m.pose.position.x = o['x']
            m.pose.position.y = o['y']
            m.pose.position.z = 0.5  # 높이의 절반만큼 올림 (바닥에서 중심까지)
            
            # 원통 방향 (기본값: 수직)
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            
            # 원통 크기 설정
            m.scale.x = 0.5  # 지름 (X축 직경)
            m.scale.y = 0.5  # 지름 (Y축 직경)
            m.scale.z = 1.0  # 높이
            
            # 노란색 설정
            m.color.r = 1.0  # 빨간색
            m.color.g = 1.0  # 초록색
            m.color.b = 0.0  # 파란색 (결과: 노란색)
            m.color.a = 1.0  # 투명도 (불투명)
            
            m.lifetime = Duration(seconds=0.0).to_msg()  # 지속적 표시
            
            ma.markers.append(m)
        
        self.get_logger().debug(f'퍼블리시 장애물: {len(ma.markers)} 개')
        self.obstacle_map_pub.publish(ma)
        


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()