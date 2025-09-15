#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleMap(Node):
    def __init__(self):
        super().__init__('obstacle_map_node')

        # 파라미터 설정 (성능 최적화)
        self.declare_parameter('merge_radius', 0.4)
        self.declare_parameter('smoothing_alpha', 0.9)      # 새로운 데이터 반영 비율
        self.declare_parameter('max_age_sec', 10.0)         # obstacle_map 마커 최대 수명
        self.declare_parameter('use_latest_tf', True)      # 최신 TF 사용
        self.declare_parameter('tf_timeout_ms', 50)        # 짧은 타임아웃

        self.merge_radius = float(self.get_parameter('merge_radius').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.max_age_sec = float(self.get_parameter('max_age_sec').value)
        self.use_latest_tf = bool(self.get_parameter('use_latest_tf').value)
        self.tf_timeout_ms = int(self.get_parameter('tf_timeout_ms').value)

        # TF 초기화 (큰 캐시)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 추적된 장애물 저장소
        self.tracked = []
        
        # TF 캐시 (성능 최적화)
        self.cached_tf = None
        self.cached_tf_time = None
        self.tf_cache_duration = 0.1  # 100ms 캐시

        # 구독자/퍼블리셔 설정
        self.obstacle_sub = self.create_subscription(
            MarkerArray, '/sensor_fusion/obstacles', self.obstacle_cb, 10
        )
        self.obstacle_map_pub = self.create_publisher(MarkerArray, '/obstacle_map', 10)

        # 주기적 퍼블리시 (더 높은 주파수)
        self.timer_pub = self.create_timer(0.1, self.publish_obstacles)  # 10Hz

        # 성능 통계
        self.transform_count = 0
        self.transform_fail_count = 0
        self.last_stats_time = self.get_clock().now()

    def obstacle_cb(self, msg: MarkerArray):
        start_time = self.get_clock().now()
        now = start_time
        points_map = []

        # 빠른 검증
        # if not msg.markers:
        #     self._remove_old(now)
        #     return

        mk = msg.markers[0]
        
        # **핵심 최적화 1: TF 한 번만 조회**
        transform = self._get_transform_optimized(mk.header.frame_id or 'velodyne')
        if transform is None:
            self.get_logger().warn("TF 변환 실패 - 데이터 건너뜀")
            return

        # **핵심 최적화 2: 배치 변환**
        if mk.points:
            points_map = self._transform_points_batch(mk.points, transform)

        if not points_map:
            self._remove_old(now)
            return

        # **핵심 최적화 3: 벡터화된 거리 계산**
        self._update_tracked_obstacles_fast(points_map, now)
        
        # 성능 통계 업데이트
        processing_time = (self.get_clock().now() - start_time).nanoseconds * 1e-6
        if processing_time > 10.0:  # 10ms 초과시 경고
            self.get_logger().warn(f"처리 시간 지연: {processing_time:.1f}ms")

    def _get_transform_optimized(self, source_frame: str):
        """최적화된 TF 조회 (캐싱 + 폴백)"""
        now = self.get_clock().now()
        
        # 캐시 확인
        if (self.cached_tf is not None and self.cached_tf_time is not None and
            (now - self.cached_tf_time).nanoseconds * 1e-9 < self.tf_cache_duration):
            return self.cached_tf
        
        try:
            if self.use_latest_tf:
                # 최신 TF 사용 (가장 빠름)
                tf = self.tf_buffer.lookup_transform(
                    'map', source_frame, rclpy.time.Time(),
                    timeout=Duration(nanoseconds=self.tf_timeout_ms * 1e6)
                )
            else:
                # 현재 시간으로 TF 조회
                tf = self.tf_buffer.lookup_transform(
                    'map', source_frame, now,
                    timeout=Duration(nanoseconds=self.tf_timeout_ms * 1e6)
                )
            
            # 캐시 업데이트
            self.cached_tf = tf
            self.cached_tf_time = now
            return tf
            
        except Exception as e:
            # 폴백: 최신 TF로 재시도 (한 번만)
            if not self.use_latest_tf:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        'map', source_frame, rclpy.time.Time(),
                        timeout=Duration(nanoseconds=20 * 1e6)  # 20ms만 대기
                    )
                    self.cached_tf = tf
                    self.cached_tf_time = now
                    return tf
                except:
                    pass
            
            self.transform_fail_count += 1
            self.get_logger().debug(f'TF 조회 실패: {e}')
            return None

    # def _transform_points_batch(self, points, transform):
    #     """배치로 포인트들을 한 번에 변환 (직접 velodyne → map)"""
    #     if not points:
    #         return []
        
    #     # TF에서 변환 행렬 추출
    #     t = transform.transform.translation
    #     r = transform.transform.rotation
        
    #     # 쿼터니언 → 회전행렬
    #     x, y, z, w = r.x, r.y, r.z, r.w
    #     R = np.array([
    #         [1-2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
    #         [2*(x*y + z*w), 1-2*(x*x + z*z), 2*(y*z - x*w)],
    #         [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x + y*y)]
    #     ])
        
    #     # 평행이동
    #     T = np.array([t.x, t.y, t.z])
        
    #     # velodyne → map
    #     points_map = []
    #     for p in points:
    #         velodyne_point = np.array([
    #             float(p.x),  
    #             float(p.y),
    #             float(p.z)
    #         ])
            
    #         # 직접 velodyne → map 변환
    #         map_point = R @ velodyne_point + T
    #         points_map.append((map_point[0], map_point[1], map_point[2]))
        
    #     self.transform_count += len(points)
    #     return points_map
    
    def _transform_points_batch(self, points, transform):
        """배치로 포인트들을 한 번에 변환 (numpy 벡터화)"""
        if not points:
            return []

        # TF에서 변환 행렬 추출
        t = transform.transform.translation
        r = transform.transform.rotation

        # 쿼터니언 → 회전행렬
        x, y, z, w = r.x, r.y, r.z, r.w
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        T = np.array([t.x, t.y, t.z])

        # 입력 포인트를 (N,3) numpy 배열로 변환
        pts = np.array([[p.x, p.y, p.z] for p in points], dtype=np.float32)  # (N,3)

        # 벡터화된 변환 (map = R * lidar + T)
        pts_map = (R @ pts.T).T + T  # (N,3)

        self.transform_count += pts.shape[0]
        return pts_map.tolist()  # 제어에 전달하기 쉽게 list로 반환


    def _update_tracked_obstacles_fast(self, points_map, now):
        """벡터화된 장애물 업데이트"""
        if not self.tracked:
            # 첫 번째 경우: 바로 추가
            for x, y, z in points_map:
                self.tracked.append({'x': x, 'y': y, 'last_seen': now})
            return
        
        # 기존 장애물 위치를 NumPy 배열로 변환
        existing_points = np.array([[o['x'], o['y']] for o in self.tracked])
        
        for x_map, y_map, _ in points_map:
            new_point = np.array([x_map, y_map])
            
            # 벡터화된 거리 계산
            distances = np.sum((existing_points - new_point) ** 2, axis=1)
            min_idx = np.argmin(distances)
            min_distance = distances[min_idx]
            
            if min_distance <= self.merge_radius ** 2:
                # 기존 장애물 업데이트 (EMA)
                idx = min_idx
                ox = self.tracked[idx]['x']
                oy = self.tracked[idx]['y']
                a = self.smoothing_alpha
                self.tracked[idx]['x'] = (1.0 - a) * ox + a * x_map
                self.tracked[idx]['y'] = (1.0 - a) * oy + a * y_map
                self.tracked[idx]['last_seen'] = now
                
                # 기존 포인트 배열 업데이트
                existing_points[idx] = [self.tracked[idx]['x'], self.tracked[idx]['y']]
            else:
                # 새 장애물 추가
                self.tracked.append({'x': x_map, 'y': y_map, 'last_seen': now})
                existing_points = np.vstack([existing_points, [x_map, y_map]])

        # 오래된 장애물 제거
        self._remove_old(now)

    def _remove_old(self, now):
        """오래된 장애물 제거 (최적화)"""
        if not self.tracked:
            return
            
        kept = []
        max_age_ns = self.max_age_sec * 1e9
        
        for o in self.tracked:
            age_ns = (now - o['last_seen']).nanoseconds
            if age_ns <= max_age_ns:
                kept.append(o)
        
        if len(kept) != len(self.tracked):
            removed_count = len(self.tracked) - len(kept)
            self.tracked = kept
            self.get_logger().debug(f'{removed_count}개 장애물 만료 제거')

    def publish_obstacles(self):
        """최적화된 퍼블리시"""
        if not self.tracked:
            # 빈 MarkerArray 퍼블리시 (이전 마커들 정리)
            ma = MarkerArray()
            delete_marker = Marker()
            delete_marker.header.frame_id = 'map'
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = 'obstacle_map'
            delete_marker.action = Marker.DELETEALL
            ma.markers.append(delete_marker)
            self.obstacle_map_pub.publish(ma)
            return
        
        ma = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # 배치로 마커 생성
        for i, o in enumerate(self.tracked):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = current_time
            m.ns = 'obstacle_map'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            
            # 위치
            m.pose.position.x = o['x']
            m.pose.position.y = o['y']
            m.pose.position.z = 0.5
            
            # 방향 (미리 설정)
            m.pose.orientation.w = 1.0
            
            # 크기
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 1.0
            
            # 색상 (노란색)
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            
            ma.markers.append(m)
        
        # 성능 통계 출력 (1초마다)
        now = self.get_clock().now()
        if (now - self.last_stats_time).nanoseconds * 1e-9 > 1.0:
            success_rate = (self.transform_count / (self.transform_count + self.transform_fail_count + 1e-6)) * 100
            self.get_logger().info(f'TF 성공률: {success_rate:.1f}% | 추적 중: {len(self.tracked)}개')
            self.last_stats_time = now
            self.transform_count = 0
            self.transform_fail_count = 0
        
        self.obstacle_map_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()