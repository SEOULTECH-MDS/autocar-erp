#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from geometry_msgs.msg import PoseArray, PointStamped, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool


class DeliverySignMap(Node):
    def __init__(self):
        super().__init__('deliverysign_map_node')

        # 파라미터 설정 (장애물 노드와 동일한 최적화)
        self.declare_parameter('merge_radius', 0.8)  # 표지판은 더 큰 반경으로 병합
        self.declare_parameter('smoothing_alpha', 0.7)      # 새로운 데이터 반영 비율
        self.declare_parameter('max_age_sec', 15.0)         # 표지판 마커 최대 수명 (더 길게)
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

        # 추적된 배달 표지판 저장소
        self.tracked_signs = []
        
        # TF 캐시 (성능 최적화)
        self.cached_tf = None
        self.cached_tf_time = None
        self.tf_cache_duration = 0.1  # 100ms 캐시

        # 구독자/퍼블리셔 설정
        self.deliverysign_sub = self.create_subscription(
            PoseArray, '/deliverysign_spot', self.deliverysign_cb, 10
        )

        # 변환된 map 좌표 표지판 위치 퍼블리시
        self.deliverysign_map_pub = self.create_publisher(PoseArray, '/deliverysign_map', 10)
        # 시각화용 마커 퍼블리시 (빨간색)
        self.deliverysign_marker_pub = self.create_publisher(MarkerArray, '/deliverysign_markers', 10)

        # 주기적 퍼블리시 (장애물 노드와 동일)
        self.timer_pub = self.create_timer(0.05, self.publish_deliverysigns)  # 20Hz

        # 성능 통계
        self.transform_count = 0
        self.transform_fail_count = 0
        self.last_stats_time = self.get_clock().now()

        self.get_logger().info('배달 표지판 맵 변환 노드 시작됨')

    def deliverysign_cb(self, msg: PoseArray):
        """라이다 좌표계의 표지판 위치를 받아서 map 좌표계로 변환"""
        start_time = self.get_clock().now()
        now = start_time

        if not msg.poses:
            self._remove_old_signs(now)
            return

        # **핵심 최적화 1: TF 한 번만 조회**
        transform = self._get_transform_optimized(msg.header.frame_id or 'velodyne')
        if transform is None:
            self.get_logger().warn("TF 변환 실패 - 표지판 데이터 건너뜀")
            return

        # **핵심 최적화 2: 배치 변환**
        signs_map = self._transform_poses_batch(msg.poses, transform)

        if not signs_map:
            self._remove_old_signs(now)
            return

        # **핵심 최적화 3: 벡터화된 거리 계산**
        self._update_tracked_signs_fast(signs_map, now)
        
        # 성능 통계 업데이트
        processing_time = (self.get_clock().now() - start_time).nanoseconds * 1e-6
        if processing_time > 10.0:  # 10ms 초과시 경고
            self.get_logger().warn(f"표지판 처리 시간 지연: {processing_time:.1f}ms")

    def _get_transform_optimized(self, source_frame: str):
        """최적화된 TF 조회 (캐싱 + 폴백) - 장애물 노드와 동일"""
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
            self.get_logger().debug(f'표지판 TF 조회 실패: {e}')
            return None

    def _transform_poses_batch(self, poses, transform):
        """배치로 Pose들을 한 번에 변환 (numpy 벡터화)"""
        if not poses:
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
        pts = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in poses], dtype=np.float32)

        # 벡터화된 변환 (map = R * lidar + T)
        pts_map = (R @ pts.T).T + T  # (N,3)

        self.transform_count += pts.shape[0]
        return pts_map.tolist()  # 제어에 전달하기 쉽게 list로 반환

    def _update_tracked_signs_fast(self, signs_map, now):
        """1개 표지판만 추적 - 가장 최신 위치 사용"""
        if not signs_map:
            return
            
        # 가장 최근에 감지된 표지판 위치 선택 (첫 번째 것 사용)
        latest_sign_pos = signs_map[0]  # 가장 최신 데이터
        x_map, y_map, z_map = latest_sign_pos[0], latest_sign_pos[1], latest_sign_pos[2]
        
        if not self.tracked_signs:
            # 첫 번째 표지판 추가
            sign = {
                'x': x_map, 
                'y': y_map, 
                'z': z_map,
                'last_seen': now
            }
            self.tracked_signs.append(sign)
            self.get_logger().info(f"새 배달 표지판 감지: ({x_map:.2f}, {y_map:.2f})")
            return
        
        # 기존 표지판과의 거리 확인
        existing_sign = self.tracked_signs[0]
        distance_sq = (existing_sign['x'] - x_map)**2 + (existing_sign['y'] - y_map)**2
        
        if distance_sq <= self.merge_radius ** 2:
            # 기존 표지판 위치 업데이트 (EMA 필터링)
            a = self.smoothing_alpha
            self.tracked_signs[0]['x'] = (1.0 - a) * existing_sign['x'] + a * x_map
            self.tracked_signs[0]['y'] = (1.0 - a) * existing_sign['y'] + a * y_map
            self.tracked_signs[0]['z'] = (1.0 - a) * existing_sign['z'] + a * z_map
            self.tracked_signs[0]['last_seen'] = now
            self.get_logger().debug(f"표지판 위치 업데이트: ({self.tracked_signs[0]['x']:.2f}, {self.tracked_signs[0]['y']:.2f})")
        else:
            # 기존 표지판을 새 위치로 완전 교체 (너무 멀리 떨어진 경우)
            self.tracked_signs[0] = {
                'x': x_map, 'y': y_map, 'z': z_map, 'last_seen': now
            }
            self.get_logger().info(f"표지판 위치 변경: ({x_map:.2f}, {y_map:.2f})")

        # 1개만 유지하므로 추가 정리 불필요

    def _remove_old_signs(self, now):
        """1개 표지판의 수명 확인"""
        if not self.tracked_signs:
            return
            
        max_age_ns = self.max_age_sec * 1e9
        sign = self.tracked_signs[0]
        age_ns = (now - sign['last_seen']).nanoseconds
        
        if age_ns > max_age_ns:
            self.tracked_signs.clear()
            self.get_logger().info("표지판 수명 만료로 제거됨")

    def publish_deliverysigns(self):
        """최적화된 표지판 퍼블리시 (PoseArray + 빨간색 마커)"""
        current_time = self.get_clock().now().to_msg()
        
        if not self.tracked_signs:
            # 빈 MarkerArray 퍼블리시 (이전 마커들 정리)
            ma = MarkerArray()
            delete_marker = Marker()
            delete_marker.header.frame_id = 'map'
            delete_marker.header.stamp = current_time
            delete_marker.ns = 'deliverysign_map'
            delete_marker.action = Marker.DELETEALL
            ma.markers.append(delete_marker)
            self.deliverysign_marker_pub.publish(ma)
            
            # 빈 PoseArray도 퍼블리시
            pose_array = PoseArray()
            pose_array.header.frame_id = 'map'
            pose_array.header.stamp = current_time
            self.deliverysign_map_pub.publish(pose_array)
            return
        
        # PoseArray 생성 (map 좌표계 표지판 위치)
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = current_time
        
        # MarkerArray 생성 (시각화용 빨간색 마커)
        ma = MarkerArray()
        
        for i, sign in enumerate(self.tracked_signs):
            # PoseArray에 추가
            pose = Pose()
            pose.position.x = sign['x']
            pose.position.y = sign['y']
            pose.position.z = sign['z']
            pose.orientation.w = 1.0  # 기본 방향
            pose_array.poses.append(pose)
            
            # 시각화 마커 생성 (빨간색 큐브)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = current_time
            marker.ns = 'deliverysign_map'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 위치
            marker.pose.position.x = sign['x']
            marker.pose.position.y = sign['y']
            marker.pose.position.z = sign['z']
            
            # 방향 (미리 설정)
            marker.pose.orientation.w = 1.0
            
            # 크기 (표지판 크기)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.6
            
            # 빨간색으로 설정
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            ma.markers.append(marker)
        
        # 퍼블리시
        self.deliverysign_map_pub.publish(pose_array)
        self.deliverysign_marker_pub.publish(ma)
        
        # 성능 통계 출력 (5초마다)
        now = self.get_clock().now()
        if (now - self.last_stats_time).nanoseconds * 1e-9 > 5.0:
            success_rate = (self.transform_count / (self.transform_count + self.transform_fail_count + 1e-6)) * 100
            status = "추적 중" if self.tracked_signs else "미감지"
            if self.tracked_signs:
                sign = self.tracked_signs[0]
                self.get_logger().info(f'표지판 TF 성공률: {success_rate:.1f}% | 상태: {status} at ({sign["x"]:.2f}, {sign["y"]:.2f})')
            else:
                self.get_logger().info(f'표지판 TF 성공률: {success_rate:.1f}% | 상태: {status}')
            self.last_stats_time = now
            self.transform_count = 0
            self.transform_fail_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = DeliverySignMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
