#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from planning_msgs.msg import ObstacleArray, Obstacle

import tf2_ros


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → Yaw 라디안."""
    # ROS 표준 ZYX (yaw-pitch-roll)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ScanToConesNode(Node):
    """
    LaserScan(/scan) → 간단 클러스터링 → /cones_obstacles(ObstacleArray, map 프레임)

    - 입력: /scan (LaserScan), TF 트리 (source_frame → map 변환)
    - 출력:
        /cones_obstacles (ObstacleArray, frame_id=map)  # 주차 구역 판단 노드 입력용
        /cones_markers   (MarkerArray, frame_id=map)    # 시각화용(옵션)

    단순한 유클리드 클러스터링(O(N^2) 근사)로 소수의 콘을 분리하는 테스트 용도입니다.
    """

    def __init__(self) -> None:
        super().__init__('scan_to_cones')

        # 파라미터
        self.declare_parameter('input_scan', '/scan')
        self.declare_parameter('output_obstacles', '/cones_obstacles')
        self.declare_parameter('output_markers', '/cones_markers')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('max_range_m', 20.0)
        self.declare_parameter('downsample_step', 1)
        self.declare_parameter('cluster_eps_m', 0.35)
        self.declare_parameter('cluster_min_pts', 3)
        self.declare_parameter('cone_radius_m', 0.15)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('tf_use_latest_on_fail', True)

        # 퍼블리셔
        self._obstacles_pub = self.create_publisher(ObstacleArray, str(self.get_parameter('output_obstacles').value), 10)
        self._markers_pub = self.create_publisher(MarkerArray, str(self.get_parameter('output_markers').value), 10)

        # TF 버퍼/리스너
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 구독자
        self.create_subscription(LaserScan, str(self.get_parameter('input_scan').value), self._on_scan, 10)

        self.get_logger().info('Scan→Cones 변환 노드 시작 (간단 클러스터링, map 프레임 출력)')

    # ───────────────────────────── 콜백 ─────────────────────────────
    def _on_scan(self, scan: LaserScan) -> None:
        """LaserScan을 받아 점군(레이저 프레임) → map 프레임으로 변환 → 클러스터링 수행."""
        target_frame = str(self.get_parameter('target_frame').value)
        max_range = float(self.get_parameter('max_range_m').value)
        step = max(1, int(self.get_parameter('downsample_step').value))

        # 1) 스캔 → 레이저 프레임 점군(x,y)
        points_src: List[Tuple[float, float]] = []
        angle = float(scan.angle_min)
        inc = float(scan.angle_increment)
        for i in range(0, len(scan.ranges), step):
            r = float(scan.ranges[i])
            if math.isfinite(r) and 0.01 < r <= max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points_src.append((x, y))
            angle += inc * step

        if not points_src:
            return

        # 2) 레이저 프레임 → map 프레임 변환 획득
        timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        try:
            # 가능하면 요청 시각의 TF를 사용
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                scan.header.frame_id,
                scan.header.stamp,
                timeout=Duration(seconds=timeout_sec),
            )
        except Exception as e:
            # 외삽(미래/과거) 오류 시 최신 TF로 폴백
            if bool(self.get_parameter('tf_use_latest_on_fail').value):
                try:
                    tf = self._tf_buffer.lookup_transform(
                        target_frame,
                        scan.header.frame_id,
                        Time(),  # TIME_ZERO: 최신 TF
                        timeout=Duration(seconds=timeout_sec),
                    )
                    self.get_logger().warn(
                        f'TF 시각 외삽으로 최신 TF로 폴백 사용: {scan.header.frame_id}→{target_frame} ({e})'
                    )
                except Exception as e2:
                    self.get_logger().warn(f'TF 변환 실패(폴백 포함): {scan.header.frame_id}→{target_frame}: {e2}')
                    return
            else:
                self.get_logger().warn(f'TF 변환 실패: {scan.header.frame_id}→{target_frame}: {e}')
                return

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tz = float(tf.transform.translation.z)  # 사용하지 않지만 보관
        qx = float(tf.transform.rotation.x)
        qy = float(tf.transform.rotation.y)
        qz = float(tf.transform.rotation.z)
        qw = float(tf.transform.rotation.w)

        # Yaw만 사용(2D 가정)
        yaw = _quat_to_yaw(qx, qy, qz, qw)
        c = math.cos(yaw)
        s = math.sin(yaw)

        def to_map(px: float, py: float) -> Tuple[float, float]:
            # Rz(yaw) * p + t (2D)
            mx = c * px - s * py + tx
            my = s * px + c * py + ty
            return mx, my

        points_map = [to_map(x, y) for (x, y) in points_src]

        # 3) 간단 유클리드 클러스터링
        eps = float(self.get_parameter('cluster_eps_m').value)
        min_pts = max(1, int(self.get_parameter('cluster_min_pts').value))
        clusters = self._euclidean_cluster(points_map, eps)

        # 4) 중심점 산출 및 퍼블리시
        centers: List[Tuple[float, float]] = []
        for cl in clusters:
            if len(cl) >= min_pts:
                sx = 0.0
                sy = 0.0
                for (x, y) in cl:
                    sx += x
                    sy += y
                cx = sx / len(cl)
                cy = sy / len(cl)
                centers.append((cx, cy))

        self._publish_obstacles(scan, centers, target_frame)
        if bool(self.get_parameter('publish_markers').value):
            self._publish_markers(scan, centers, target_frame)

    # ───────────────────────────── 유틸 ─────────────────────────────
    def _euclidean_cluster(self, pts: List[Tuple[float, float]], eps: float) -> List[List[Tuple[float, float]]]:
        """아주 단순한 O(N^2) 근사 클러스터링 (연결 성분 방식).
        - eps 이내 이웃이 있으면 같은 군집으로 합침.
        테스트/소수 포인트용으로 충분.
        """
        if not pts:
            return []
        visited = [False] * len(pts)
        clusters: List[List[Tuple[float, float]]] = []

        for i in range(len(pts)):
            if visited[i]:
                continue
            visited[i] = True
            seed = [i]
            queue = [i]
            while queue:
                j = queue.pop()
                xj, yj = pts[j]
                for k in range(len(pts)):
                    if visited[k]:
                        continue
                    xk, yk = pts[k]
                    if (xk - xj) * (xk - xj) + (yk - yj) * (yk - yj) <= eps * eps:
                        visited[k] = True
                        queue.append(k)
                        seed.append(k)
            clusters.append([pts[idx] for idx in seed])
        return clusters

    def _publish_obstacles(self, scan: LaserScan, centers: List[Tuple[float, float]], frame: str) -> None:
        """클러스터 중심들을 ObstacleArray(map)로 퍼블리시."""
        arr = ObstacleArray()
        arr.header.frame_id = frame
        # 스캔 타임스탬프를 그대로 사용(시뮬타임 호환)
        arr.header.stamp = scan.header.stamp
        radius = float(self.get_parameter('cone_radius_m').value)
        for i, (x, y) in enumerate(centers):
            ob = Obstacle()
            ob.id = i
            ob.type = 'cone'
            ob.center.x = float(x)
            ob.center.y = float(y)
            ob.center.z = 0.0
            ob.radius = float(radius)
            arr.obstacles.append(ob)
        self._obstacles_pub.publish(arr)

    def _publish_markers(self, scan: LaserScan, centers: List[Tuple[float, float]], frame: str) -> None:
        """시각화용 MarkerArray 퍼블리시(작은 구체)."""
        ma = MarkerArray()
        for i, (x, y) in enumerate(centers):
            mk = Marker()
            mk.header.frame_id = frame
            mk.header.stamp = scan.header.stamp
            mk.ns = 'cones'
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = float(x)
            mk.pose.position.y = float(y)
            mk.pose.position.z = 0.0
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.25
            mk.scale.y = 0.25
            mk.scale.z = 0.25
            mk.color.a = 0.9
            mk.color.r = 1.0
            mk.color.g = 1.0
            mk.color.b = 0.0
            ma.markers.append(mk)
        self._markers_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanToConesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


