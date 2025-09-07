#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from planning_msgs.msg import ObstacleArray, Obstacle

import tf2_ros


def _quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> List[List[float]]:
    """쿼터니언 → 3x3 회전행렬.
    일반적인 3D 회전을 지원하여 velodyne→map 변환 시 정확도를 높입니다.
    """
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


class RubberConesAdapterNode(Node):
    """
    Sensor Fusion 출력 MarkerArray(`/sensor_fusion/rubber_cones`, frame: velodyne)
     → ObstacleArray(`/cones_obstacles`, frame: map) 브릿지 노드

    - 기존 `parking_area_node` 입력 형식을 유지하므로 다운스트림 변경 없이 연결 가능
    - TF 외삽 문제를 완화하기 위해 최신 TF 폴백 옵션 제공
    - 색상 필터링 옵션 제공(기본 all)
    - 디버그용 변환 후 마커 퍼블리시 옵션 제공
    """

    def __init__(self) -> None:
        super().__init__('rubber_cones_adapter')

        # 파라미터
        self.declare_parameter('input_markers', '/sensor_fusion/tracked_rubber_cones')
        self.declare_parameter('output_obstacles', '/cones_obstacles')
        self.declare_parameter('output_markers', '/cones_markers_from_rubber')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('cone_radius_m', 0.15)
        self.declare_parameter('publish_markers', True)
        # time/TF
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('tf_use_latest_on_fail', True)
        # color_filter: all | yellow | blue | white
        self.declare_parameter('color_filter', 'all')
        # marker persistence (accumulate points in target_frame for RViz)
        self.declare_parameter('marker_persist', True)
        self.declare_parameter('marker_merge_radius', 0.25)
        # vehicle odometry & side labeling
        self.declare_parameter('odom_topic', '/autocar/location')
        self.declare_parameter('enable_side_label', True)
        # vehicle pose publish & visualization
        self.declare_parameter('publish_vehicle_pose', True)
        self.declare_parameter('vehicle_pose_topic', '/vehicle_pose_in_target')
        self.declare_parameter('publish_vehicle_marker', True)
        self.declare_parameter('vehicle_marker_topic', '/vehicle_marker')

        # 퍼블리셔
        self._obstacles_pub = self.create_publisher(
            ObstacleArray, str(self.get_parameter('output_obstacles').value), 10
        )
        self._markers_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('output_markers').value), 10
        )
        # vehicle publishers (lazy used)
        self._vehicle_odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('vehicle_pose_topic').value), 10
        )
        self._vehicle_marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('vehicle_marker_topic').value), 10
        )

        # TF 버퍼/리스너
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # internal state
        self._last_odom = None
        self._persist_marker_points: List[Tuple[float, float]] = []  # [(x,y)]

        # 구독자
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter('input_markers').value),
            self._on_markers,
            10,
        )

        # vehicle odometry subscription
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            10,
        )

        self.get_logger().info('Rubber cones → ObstacleArray 어댑터 노드 시작')

    # ───────────────────────────── 콜백 ─────────────────────────────
    def _on_markers(self, msg: MarkerArray) -> None:
        target_frame = str(self.get_parameter('target_frame').value)
        timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        # 입력 마커가 없으면 무시
        if not msg.markers:
            return

        # TF 획득 (가능하면 입력 메시지의 타임스탬프 사용)
        src_frame = msg.markers[0].header.frame_id if msg.markers else 'velodyne'
        stamp = msg.markers[0].header.stamp if msg.markers else self.get_clock().now().to_msg()
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                src_frame,
                stamp,
                timeout=Duration(seconds=timeout_sec),
            )
        except Exception as e:
            if bool(self.get_parameter('tf_use_latest_on_fail').value):
                try:
                    tf = self._tf_buffer.lookup_transform(
                        target_frame,
                        src_frame,
                        Time(),  # 최신 TF
                        timeout=Duration(seconds=timeout_sec),
                    )
                    self.get_logger().warn(
                        f'TF 시각 외삽으로 최신 TF로 폴백 사용: {src_frame}→{target_frame} ({e})'
                    )
                except Exception as e2:
                    self.get_logger().warn(f'TF 변환 실패(폴백 포함): {src_frame}→{target_frame}: {e2}')
                    return
            else:
                self.get_logger().warn(f'TF 변환 실패: {src_frame}→{target_frame}: {e}')
                return

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tz = float(tf.transform.translation.z)
        qx = float(tf.transform.rotation.x)
        qy = float(tf.transform.rotation.y)
        qz = float(tf.transform.rotation.z)
        qw = float(tf.transform.rotation.w)
        R = _quat_to_rot_matrix(qx, qy, qz, qw)

        def apply_tf(px: float, py: float, pz: float) -> Tuple[float, float, float]:
            rx = R[0][0] * px + R[0][1] * py + R[0][2] * pz + tx
            ry = R[1][0] * px + R[1][1] * py + R[1][2] * pz + ty
            rz = R[2][0] * px + R[2][1] * py + R[2][2] * pz + tz
            return rx, ry, rz

        # 색상 필터 준비
        cfilter = str(self.get_parameter('color_filter').value).lower().strip()

        def accept_color(m: Marker) -> bool:
            if cfilter == 'all':
                return True
            # make_marker에서 r,g,b가 정확히 지정되므로 근사 비교
            r, g, b = float(m.color.r), float(m.color.g), float(m.color.b)
            if cfilter == 'yellow':
                return abs(r - 1.0) < 1e-3 and abs(g - 1.0) < 1e-3 and abs(b - 0.0) < 1e-3
            if cfilter == 'blue':
                return abs(r - 0.0) < 1e-3 and abs(g - 0.0) < 1e-3 and abs(b - 1.0) < 1e-3
            if cfilter == 'white':
                return abs(r - 1.0) < 1e-3 and abs(g - 1.0) < 1e-3 and abs(b - 1.0) < 1e-3
            return True

        # 1) 입력 포인트들을 target 프레임으로 변환
        pts_map: List[Tuple[float, float, float]] = []
        for mk in msg.markers:
            if not accept_color(mk):
                continue
            # Marker.pose는 기본값(w=1)이고 points에 좌표가 담김
            for p in mk.points:
                x, y, z = float(p.x), float(p.y), float(p.z)
                mx, my, mz = apply_tf(x, y, z)
                pts_map.append((mx, my, mz))

        # 2) 차량 기준 좌/우 라벨링
        labels = None
        if pts_map and bool(self.get_parameter('enable_side_label').value):
            vpose = self._get_vehicle_pose_in_target(target_frame)
            if vpose is not None:
                labels = self._label_points_vehicle_side(pts_map, vpose)

        # 3) ObstacleArray로 퍼블리시 (z는 무시하고 2D로 투영)
        if pts_map:
            self._publish_obstacles(pts_map, target_frame, stamp, labels)
            if bool(self.get_parameter('publish_markers').value):
                self._publish_markers(pts_map, target_frame, stamp)
        # 4) vehicle pose republish/visualize in target frame
        if bool(self.get_parameter('publish_vehicle_pose').value):
            self._maybe_publish_vehicle_pose(target_frame, stamp)
        if bool(self.get_parameter('publish_vehicle_marker').value):
            self._maybe_publish_vehicle_marker(target_frame, stamp)

    # ───────────────────────────── 퍼블리시 유틸 ─────────────────────────────
    def _publish_obstacles(self, points: List[Tuple[float, float, float]], frame: str, stamp, labels: List[str] = None) -> None:
        arr = ObstacleArray()
        arr.header.frame_id = frame
        arr.header.stamp = stamp
        radius = float(self.get_parameter('cone_radius_m').value)
        for i, (x, y, _z) in enumerate(points):
            ob = Obstacle()
            ob.id = i
            ob.type = 'cone'
            ob.center.x = float(x)
            ob.center.y = float(y)
            ob.center.z = 0.0
            ob.radius = float(radius)
            if labels is not None and i < len(labels) and labels[i] is not None:
                ob.description = labels[i]
            arr.obstacles.append(ob)
        self._obstacles_pub.publish(arr)

    def _publish_markers(self, points: List[Tuple[float, float, float]], frame: str, stamp) -> None:
        # accumulate if enabled
        if bool(self.get_parameter('marker_persist').value):
            mr = float(self.get_parameter('marker_merge_radius').value)
            mr2 = mr * mr
            for (x, y, _z) in points:
                px = float(x); py = float(y)
                best_i = -1; best_d2 = 1e9
                for i, (qx, qy) in enumerate(self._persist_marker_points):
                    dx = qx - px; dy = qy - py
                    d2 = dx*dx + dy*dy
                    if d2 < best_d2:
                        best_d2 = d2; best_i = i
                if best_d2 <= mr2 and best_i >= 0:
                    qx, qy = self._persist_marker_points[best_i]
                    self._persist_marker_points[best_i] = (0.7*qx + 0.3*px, 0.7*qy + 0.3*py)
                else:
                    self._persist_marker_points.append((px, py))
        ma = MarkerArray()
        mk = Marker()
        mk.header.frame_id = frame
        mk.header.stamp = stamp
        mk.ns = 'rubber_cones_map'
        mk.id = 0
        mk.type = Marker.POINTS
        mk.action = Marker.ADD
        mk.scale.x = 0.2
        mk.scale.y = 0.2
        mk.color.a = 1.0
        mk.color.r = 1.0
        mk.color.g = 1.0
        mk.color.b = 0.0
        mk.pose.orientation.w = 1.0
        from geometry_msgs.msg import Point
        if bool(self.get_parameter('marker_persist').value):
            src = [(x, y, 0.0) for (x, y) in self._persist_marker_points]
        else:
            src = points
        for (x, y, z) in src:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            mk.points.append(p)
        ma.markers.append(mk)
        self._markers_pub.publish(ma)

    # ───────────────────────────── vehicle pose & labeling ─────────────────────────────
    def _on_odom(self, odom: Odometry) -> None:
        self._last_odom = odom

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_vehicle_pose_in_target(self, target_frame: str):
        if self._last_odom is None:
            return None
        od = self._last_odom
        ox, oy, oz = float(od.pose.pose.position.x), float(od.pose.pose.position.y), float(od.pose.pose.position.z)
        qx, qy, qz, qw = float(od.pose.pose.orientation.x), float(od.pose.pose.orientation.y), float(od.pose.pose.orientation.z), float(od.pose.pose.orientation.w)
        src = od.header.frame_id if od.header.frame_id else target_frame
        if src == target_frame:
            return (ox, oy, self._yaw_from_quat(qx, qy, qz, qw))
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame, src, od.header.stamp,
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target_frame, src, Time(),
                    timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
                )
            except Exception:
                return None
        tx, ty, tz = float(tf.transform.translation.x), float(tf.transform.translation.y), float(tf.transform.translation.z)
        tqx, tqy, tqz, tqw = float(tf.transform.rotation.x), float(tf.transform.rotation.y), float(tf.transform.rotation.z), float(tf.transform.rotation.w)
        R = _quat_to_rot_matrix(tqx, tqy, tqz, tqw)
        vx = R[0][0] * ox + R[0][1] * oy + R[0][2] * oz + tx
        vy = R[1][0] * ox + R[1][1] * oy + R[1][2] * oz + ty
        # orientation: use quaternion multiplication q_tf * q_odom
        qmx, qmy, qmz, qmw = self._quat_multiply((tqx, tqy, tqz, tqw), (qx, qy, qz, qw))
        yaw = self._yaw_from_quat(qmx, qmy, qmz, qmw)
        return (vx, vy, yaw)

    def _label_points_vehicle_side(self, pts: List[Tuple[float, float, float]], vehicle_pose) -> List[str]:
        vx, vy, vyaw = vehicle_pose
        cos_y = math.cos(vyaw)
        sin_y = math.sin(vyaw)
        labels: List[str] = []
        for (px, py, _pz) in pts:
            dx = float(px) - vx
            dy = float(py) - vy
            x_rel =  cos_y * dx + sin_y * dy
            y_rel = -sin_y * dx + cos_y * dy
            if y_rel > 0.05:
                labels.append('left')
            elif y_rel < -0.05:
                labels.append('right')
            else:
                labels.append('center')
        return labels

    @staticmethod
    def _quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        )

    def _maybe_publish_vehicle_pose(self, target_frame: str, stamp) -> None:
        if self._last_odom is None:
            return
        od_in = self._last_odom
        vpose = self._get_vehicle_pose_in_target(target_frame)
        if vpose is None:
            return
        vx, vy, vyaw = vpose
        # build odometry in target
        od = Odometry()
        od.header.frame_id = target_frame
        od.header.stamp = stamp
        od.pose.pose.position.x = float(vx)
        od.pose.pose.position.y = float(vy)
        od.pose.pose.position.z = float(od_in.pose.pose.position.z)
        # yaw to quaternion
        half = vyaw * 0.5
        od.pose.pose.orientation.x = 0.0
        od.pose.pose.orientation.y = 0.0
        od.pose.pose.orientation.z = math.sin(half)
        od.pose.pose.orientation.w = math.cos(half)
        # keep original twist if any
        od.twist = od_in.twist
        self._vehicle_odom_pub.publish(od)

    def _maybe_publish_vehicle_marker(self, target_frame: str, stamp) -> None:
        vpose = self._get_vehicle_pose_in_target(target_frame)
        if vpose is None:
            return
        vx, vy, vyaw = vpose
        ma = MarkerArray()
        mk = Marker()
        mk.header.frame_id = target_frame
        mk.header.stamp = stamp
        mk.ns = 'vehicle'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.scale.x = 1.2
        mk.scale.y = 0.3
        mk.scale.z = 0.3
        mk.color.a = 1.0
        mk.color.r = 0.2
        mk.color.g = 0.8
        mk.color.b = 0.2
        mk.pose.position.x = float(vx)
        mk.pose.position.y = float(vy)
        mk.pose.position.z = 0.0
        half = vyaw * 0.5
        mk.pose.orientation.x = 0.0
        mk.pose.orientation.y = 0.0
        mk.pose.orientation.z = math.sin(half)
        mk.pose.orientation.w = math.cos(half)
        ma.markers.append(mk)
        self._vehicle_marker_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RubberConesAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


