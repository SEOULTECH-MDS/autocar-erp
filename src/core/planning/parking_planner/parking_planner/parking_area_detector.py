#!/usr/bin/env python3
from typing import Dict, List, Tuple, Optional, Set

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from planning_msgs.msg import ObstacleArray

import tf2_ros


class ParkingAreaDetector(Node):
    def __init__(self) -> None:
        super().__init__('parking_area_detector')
        # Parameters
        self.declare_parameter('input_topic', '/parking/cones_mapped_in_roi')
        self.declare_parameter('tf_timeout_sec', 0.5)

        # ROI in base_link: y in [roi_y_min_m, roi_y_max_m]
        self.declare_parameter('roi_y_min_m', -3.0)
        self.declare_parameter('roi_y_max_m', 0.0)

        # Graph connectivity distance (m) for clustering along a line
        self.declare_parameter('connect_distance_m', 1.8)

        # Debug
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('all_cones_topic', '/parking/debugging_all_cones')
        self.declare_parameter('roi_topic', '/parking/debugging_roi')
        self.declare_parameter('roi_cones_topic', '/parking/debugging_promoted_cones')
        self.declare_parameter('line_topic', '/parking/debugging_line')
        # removed: viz_rate_hz (timer-based viz disabled)
        self.declare_parameter('promoted_markers_topic', '/parking/cones_promoted_markers')
        # Open-area detection params
        self.declare_parameter('open_min_length_m', 4.75)
        self.declare_parameter('open_clear_distance_m', 3.0)
        self.declare_parameter('open_lateral_tol_m', 0.5)
        self.declare_parameter('area2_length_m', 9.5)
        self.declare_parameter('max_area_length_m', 17.0)
        self.declare_parameter('parking_pose_offset_y', 1.2)
        self.declare_parameter('parking_pose_offset_x', 1.3)
        self.declare_parameter('parking_pose_offset_estimated_x', 0.8)

        qos = QoSProfile(depth=10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        inp = str(self.get_parameter('input_topic').value)
        self.sub = self.create_subscription(ObstacleArray, inp, self._on_cones, qos)
        # Optional: subscribe to promoted markers to stabilize reference even if obstacles briefly drop
        prom_mk_topic = str(self.get_parameter('promoted_markers_topic').value)
        self.sub_prom = self.create_subscription(MarkerArray, prom_mk_topic, self._on_promoted_markers, qos)

        # Publishers
        self.pub_all = self.create_publisher(MarkerArray, str(self.get_parameter('all_cones_topic').value), qos)
        self.pub_roi = self.create_publisher(MarkerArray, str(self.get_parameter('roi_topic').value), qos)
        self.pub_roi_cones = self.create_publisher(MarkerArray, str(self.get_parameter('roi_cones_topic').value), qos)
        self.pub_line = self.create_publisher(MarkerArray, str(self.get_parameter('line_topic').value), qos)
        # Open-area outputs
        from std_msgs.msg import Int32
        from geometry_msgs.msg import PoseStamped
        self.pub_open_idx = self.create_publisher(Int32, '/parking/open_area_idx', qos)
        self.pub_parking_pose = self.create_publisher(PoseStamped, '/parking/parking_pose', qos)

        # State: track first entry times for ROI membership
        self._roi_first_seen_sec: Dict[int, float] = {}
        self._roi_prev_ids: Set[int] = set()
        self._current_ref_id: Optional[int] = None
        # Latest promoted markers by id -> (x,y, stamp_sec)
        self._promoted_from_markers: Dict[int, Tuple[float, float, float]] = {}
        # Last merged promoted cones map: id -> (x,y)
        self._last_cones_map: Dict[int, Tuple[float, float]] = {}
        # Announce guard to avoid missing reference creation log
        self._last_announced_ref_id: Optional[int] = None
        # Area decision state: stop re-announcing/publishing once decided until reference changes
        self._area_decided: bool = False
        # One-time log guard per reference lifecycle
        self._logged_tags: Set[str] = set()

        self.get_logger().info('parking_area_detector started')

    def _publish_roi_band(self, now_msg) -> None:
        if not bool(self.get_parameter('debug_enabled').value):
            return
        y_min = float(self.get_parameter('roi_y_min_m').value)
        y_max = float(self.get_parameter('roi_y_max_m').value)
        roi_arr = MarkerArray()
        rect = Marker()
        rect.header.frame_id = 'base_link'
        # Use stamp=0 to always use the latest TF and avoid future extrapolation
        rect.header.stamp = rclpy.time.Time().to_msg()
        rect.ns = 'roi'
        rect.id = 1
        rect.type = Marker.CUBE
        rect.action = Marker.ADD
        rect.pose.position.x = 6.0
        rect.pose.position.y = (y_min + y_max) * 0.5
        rect.pose.position.z = 0.0
        rect.pose.orientation.w = 1.0
        rect.scale.x = 20.0
        rect.scale.y = abs(y_max - y_min)
        rect.scale.z = 0.05
        rect.color.r = 0.0
        rect.color.g = 0.8
        rect.color.b = 1.0
        rect.color.a = 0.25
        rect.lifetime = Duration(seconds=0.8).to_msg()
        roi_arr.markers.append(rect)
        self.pub_roi.publish(roi_arr)

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        # yaw from quaternion
        s2 = 2.0 * (qw * qz + qx * qy)
        c2 = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(s2, c2)

    def _lookup_map_to_bl(self) -> Optional[Tuple[float, float, float]]:
        timeout = float(self.get_parameter('tf_timeout_sec').value)
        try:
            # transform from map to base_link: p_bl = R(yaw) * p_map + t
            tf = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time(), timeout=Duration(seconds=timeout))
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            yaw = self._quat_to_yaw(float(tf.transform.rotation.x), float(tf.transform.rotation.y), float(tf.transform.rotation.z), float(tf.transform.rotation.w))
            return (tx, ty, yaw)
        except Exception as e:
            self.get_logger().warn(f'TF map->base_link failed: {e}')
            return None

    @staticmethod
    def _apply_2d_transform(x: float, y: float, tx: float, ty: float, yaw: float) -> Tuple[float, float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        xb = c * x - s * y + tx
        yb = s * x + c * y + ty
        return (xb, yb)

    # removed: inverse transform helper (unused)

    def _publish_delete_all(self, pub) -> None:
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        pub.publish(arr)

    def _on_promoted_markers(self, msg: MarkerArray) -> None:
        # Capture latest promoted markers (assumed map frame)
        for mk in msg.markers:
            # Mapper uses id = 100000 + track_id for promoted markers; allow direct id if not offset
            tid = int(mk.id)
            if tid >= 100000:
                tid = tid - 100000
            stamp_sec = float(mk.header.stamp.sec) + float(mk.header.stamp.nanosec) / 1e9 if mk.header.stamp else self.get_clock().now().nanoseconds / 1e9
            self._promoted_from_markers[tid] = (float(mk.pose.position.x), float(mk.pose.position.y), stamp_sec)

    def _on_cones(self, msg: ObstacleArray) -> None:
        debug = bool(self.get_parameter('debug_enabled').value)
        now_msg = self.get_clock().now().to_msg()

        # Input is promoted cones (persisting after dwell) in map frame
        obs_map: Dict[int, Tuple[float, float]] = {int(o.id): (float(o.center.x), float(o.center.y)) for o in msg.obstacles}
        # Merge with latest promoted markers to avoid brief drops
        cones_dict: Dict[int, Tuple[float, float]] = dict(obs_map)
        for tid, (mx, my, _ts) in self._promoted_from_markers.items():
            if tid not in cones_dict:
                cones_dict[tid] = (mx, my)
        cones_map: List[Tuple[int, float, float]] = [(cid, xy[0], xy[1]) for cid, xy in cones_dict.items()]
        # Save last merged set for timer-based re-publish
        self._last_cones_map = {cid: (mx, my) for (cid, mx, my) in cones_map}

        # Publish ROI band marker (timer also publishes independently)
        if debug:
            self._publish_roi_band(now_msg)

            # ROI cones in MAP frame (no DELETEALL, rely on short lifetime/same ids)
            rc_arr = MarkerArray()
            map_frame = msg.header.frame_id or 'map'
            for i, (cid, mx, my) in enumerate(cones_map):
                mk = Marker()
                mk.header.frame_id = map_frame
                mk.header.stamp = now_msg
                mk.ns = 'roi_cones_map'
                mk.id = 2000 + int(i)
                mk.type = Marker.CYLINDER
                mk.action = Marker.ADD
                mk.pose.position.x = float(mx)
                mk.pose.position.y = float(my)
                mk.pose.position.z = 0.0
                mk.pose.orientation.w = 1.0
                mk.scale.x = 0.35
                mk.scale.y = 0.35
                mk.scale.z = 0.35
                mk.color.r = 0.0
                mk.color.g = 0.0
                mk.color.b = 1.0
                mk.color.a = 0.5
                mk.lifetime = Duration(seconds=0.8).to_msg()
                rc_arr.markers.append(mk)
            if rc_arr.markers:
                self.pub_roi_cones.publish(rc_arr)

        # Note: all cones are published in map frame above; skip base_link copy to reduce clutter

        if not cones_map:
            self._roi_prev_ids = set()
            self._current_ref_id = None
            return

        # 3) 기준 콘 선택 (map 기준; TF 있으면 base_link x 기준 tie-break)
        current_ids: Set[int] = set(cid for (cid, _, _) in cones_map)
        new_ids = [cid for cid in current_ids if cid not in self._roi_prev_ids]
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for cid, mx, my in cones_map:
            if cid not in self._roi_first_seen_sec:
                self._roi_first_seen_sec[cid] = now_sec

        def x_of_map(cid: int) -> float:
            for (i, mx, my) in cones_map:
                if i == cid:
                    return float(mx)
            return float('inf')

        def promoted_since(cid: int) -> float:
            # parse from description if present: '...;promoted_since=xxx;...'
            for o in msg.obstacles:
                if int(o.id) == cid and o.description:
                    parts = str(o.description).split(';')
                    for p in parts:
                        if p.startswith('promoted_since='):
                            try:
                                return float(p.split('=')[1])
                            except Exception:
                                return float('inf')
            return float('inf')

        # base_link x는 더 이상 사용하지 않음(참조 고정으로 불필요)

        # Lock reference if still present; otherwise choose earliest promoted_since and lock
        if (self._current_ref_id is not None) and (self._current_ref_id in current_ids):
            ref_id = int(self._current_ref_id)
        else:
            ref_id = min(current_ids, key=lambda c: (promoted_since(c), c))
            self._current_ref_id = int(ref_id)

        # Always announce when a new reference is established (once per ref change)
        if self._last_announced_ref_id != ref_id:
            # reset state for new lifecycle
            self._area_decided = False
            self._logged_tags.clear()
            self.get_logger().info('reference 생성, 구역 1 탐색 시작')
            self._last_announced_ref_id = int(ref_id)

        # If area already decided for this reference, stop further processing/logging
        if self._area_decided:
            return

        def log_once(tag: str, msg: str, level: str = 'info') -> None:
            if tag in self._logged_tags:
                return
            self._logged_tags.add(tag)
            if level == 'error':
                self.get_logger().error(msg)
            else:
                self.get_logger().info(msg)

        ref_pt = next((xy for (cid, *xy) in cones_map if cid == ref_id), None)
        if ref_pt is None:
            self._roi_prev_ids = current_ids
            return
        ref_x, ref_y = float(ref_pt[0]), float(ref_pt[1])

        # Publish reference text only in map frame (avoid duplicate base_link text)
        if debug:
            arr = MarkerArray()
            txtm = Marker()
            txtm.header.frame_id = msg.header.frame_id or 'map'
            txtm.header.stamp = now_msg
            txtm.ns = 'promoted_ref_map'
            txtm.id = 9999
            txtm.type = Marker.TEXT_VIEW_FACING
            txtm.action = Marker.ADD
            txtm.pose.position.x = float(ref_x)
            txtm.pose.position.y = float(ref_y)
            txtm.pose.position.z = 1.5
            txtm.pose.orientation.w = 1.0
            txtm.scale.z = 0.4
            txtm.color.r = 1.0
            txtm.color.g = 1.0
            txtm.color.b = 1.0
            txtm.color.a = 0.95
            txtm.text = f'reference_id={ref_id}'
            txtm.lifetime = Duration(seconds=0.8).to_msg()
            arr.markers.append(txtm)
            self.pub_roi_cones.publish(arr)

        # 4) 연결 그래프 기반 클러스터링 (map 프레임)
        connect = float(self.get_parameter('connect_distance_m').value)
        pts = {cid: (mx, my) for (cid, mx, my) in cones_map}
        nbrs: Dict[int, List[int]] = {cid: [] for cid in pts.keys()}
        thresh2 = connect * connect
        cids = list(pts.keys())
        for i in range(len(cids)):
            ci = cids[i]
            xi, yi = pts[ci]
            for j in range(i + 1, len(cids)):
                cj = cids[j]
                xj, yj = pts[cj]
                d2 = (xi - xj) * (xi - xj) + (yi - yj) * (yi - yj)
                if d2 <= thresh2:
                    nbrs[ci].append(cj)
                    nbrs[cj].append(ci)

        comp: List[int] = []
        visited: Set[int] = set()
        stack: List[int] = [ref_id]
        while stack:
            c = stack.pop()
            if c in visited:
                continue
            visited.add(c)
            comp.append(c)
            for nb in nbrs.get(c, []):
                if nb not in visited:
                    stack.append(nb)

        # 직선 근사 (map 프레임 주성분)
        xs = [pts[c][0] for c in comp]
        ys = [pts[c][1] for c in comp]
        if len(xs) >= 2:
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            cx = [x - mean_x for x in xs]
            cy = [y - mean_y for y in ys]
            sxx = sum([u * u for u in cx])
            sxy = sum([u * v for (u, v) in zip(cx, cy)])
            syy = sum([v * v for v in cy])
            theta = 0.5 * math.atan2(2.0 * sxy, (sxx - syy))
            dir_c = math.cos(theta)
            dir_s = math.sin(theta)
            projs = [dir_c * (x - mean_x) + dir_s * (y - mean_y) for (x, y) in zip(xs, ys)]
            min_p = min(projs)
            max_p = max(projs)
            length = max(0.1, (max_p - min_p))
            center_x = mean_x + (min_p + max_p) * 0.5 * dir_c
            center_y = mean_y + (min_p + max_p) * 0.5 * dir_s
            yaw_line = math.atan2(dir_s, dir_c)
        else:
            center_x = xs[0]
            center_y = ys[0]
            yaw_line = 0.0
            length = 0.1

        # 4-1) 직선 시각화 (map 프레임) - CUBE + 방향 ARROW + 양끝 점
        if debug:
            arr = MarkerArray()
            # CUBE 본체
            mk = Marker()
            mk.header.frame_id = msg.header.frame_id or 'map'
            mk.header.stamp = now_msg
            mk.ns = 'cluster_line_map'
            mk.id = 1
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = float(center_x)
            mk.pose.position.y = float(center_y)
            mk.pose.position.z = 0.0
            mk.pose.orientation.z = float(math.sin(0.5 * yaw_line))
            mk.pose.orientation.w = float(math.cos(0.5 * yaw_line))
            mk.scale.x = float(max(0.5, length))
            mk.scale.y = 0.5
            mk.scale.z = 0.2
            mk.color.r = 1.0
            mk.color.g = 0.0
            mk.color.b = 0.0
            mk.color.a = 0.3
            mk.lifetime = Duration(seconds=0.8).to_msg()
            arr.markers.append(mk)

            # 방향 ARROW
            arrow = Marker()
            arrow.header.frame_id = mk.header.frame_id
            arrow.header.stamp = now_msg
            arrow.ns = 'cluster_line_map'
            arrow.id = 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.2
            arrow.scale.y = 0.4
            arrow.scale.z = 0.4
            arrow.color.r = 0.0
            arrow.color.g = 0.4
            arrow.color.b = 1.0
            arrow.color.a = 1.0
            p0 = Point(); p0.x = float(center_x - 0.5 * length * math.cos(yaw_line)); p0.y = float(center_y - 0.5 * length * math.sin(yaw_line)); p0.z = 0.0
            p1 = Point(); p1.x = float(center_x + 0.5 * length * math.cos(yaw_line)); p1.y = float(center_y + 0.5 * length * math.sin(yaw_line)); p1.z = 0.0
            arrow.points = [p0, p1]
            arrow.lifetime = Duration(seconds=0.8).to_msg()
            arr.markers.append(arrow)

            # 양끝 점 SPHERE
            for k, (px, py, mid) in enumerate([(p0.x, p0.y, 10001), (p1.x, p1.y, 10002)]):
                sp = Marker()
                sp.header.frame_id = mk.header.frame_id
                sp.header.stamp = now_msg
                sp.ns = 'cluster_line_map'
                sp.id = mid
                sp.type = Marker.SPHERE
                sp.action = Marker.ADD
                sp.pose.position.x = px
                sp.pose.position.y = py
                sp.pose.position.z = 0.0
                sp.pose.orientation.w = 1.0
                sp.scale.x = 0.5
                sp.scale.y = 0.5
                sp.scale.z = 0.5
                sp.color.r = 1.0
                sp.color.g = 0.0
                sp.color.b = 0.0
                sp.color.a = 0.3
                sp.lifetime = Duration(seconds=0.8).to_msg()
                arr.markers.append(sp)

            self.pub_line.publish(arr)

        # 4-2) Open area detection on this line (3-stage)
        try:
            open_min = float(self.get_parameter('open_min_length_m').value)
            clear_d = float(self.get_parameter('open_clear_distance_m').value)
            lat_tol = float(self.get_parameter('open_lateral_tol_m').value)
            area2_len = float(self.get_parameter('area2_length_m').value)
            max_len = float(self.get_parameter('max_area_length_m').value)
            off_y = float(self.get_parameter('parking_pose_offset_y').value)
            off_x = float(self.get_parameter('parking_pose_offset_x').value)
            off_x_est = float(self.get_parameter('parking_pose_offset_estimated_x').value)
            if len(xs) >= 2:
                # project reference and other-end
                # find which endpoint is not reference
                # compute projections of all component points to param t along the principal axis
                # axis unit vector
                ux = math.cos(yaw_line)
                uy = math.sin(yaw_line)
                # endpoint params in map frame
                t0 = -0.5 * length
                t1 = +0.5 * length
                # reference projection param
                tref = ux * (ref_x - center_x) + uy * (ref_y - center_y)
                # choose other end param
                tend = t0 if abs(t0 - tref) > abs(t1 - tref) else t1
                end_x = center_x + tend * ux
                end_y = center_y + tend * uy

                # Stage 1: reference-immediate check (only before cluster reaches open_min)
                if length < open_min:
                    # within connect distance from reference along +axis
                    near_found = False
                    for cid, (px, py) in pts.items():
                        tpr = ux * (px - ref_x) + uy * (py - ref_y)
                        if tpr >= 0.0 and tpr <= connect:
                            latp = abs(-(uy) * (px - center_x) + (ux) * (py - center_y))
                            if latp <= lat_tol:
                                near_found = True
                                break
                    if not near_found:
                        # find next cone beyond clear_d from reference for constructing line
                        next_found = None
                        next_t = float('inf')
                        for cid, (px, py) in pts.items():
                            tpr = ux * (px - ref_x) + uy * (py - ref_y)
                            if tpr >= clear_d:
                                # ignore cones closer than connect distance from reference
                                if math.hypot(px - ref_x, py - ref_y) < connect:
                                    continue
                                latp = abs(-(uy) * (px - center_x) + (ux) * (py - center_y))
                                if latp <= lat_tol and tpr < next_t:
                                    next_t = tpr
                                    next_found = (px, py)
                        if next_found is not None:
                            look_start = (ref_x, ref_y)
                            look_end = (next_found[0], next_found[1])
                            log_once('open_idx_0', 'Open Area idx: 0')
                            publish_open(0, ref_x, ref_y, yaw_line)
                            self._area_decided = True
                            return
                        # if no next cone yet, defer until more cones appear
                        return

                def point_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
                    vx, vy = bx - ax, by - ay
                    wx, wy = px - ax, py - ay
                    vv = vx * vx + vy * vy
                    if vv <= 1e-9:
                        dx, dy = px - ax, py - ay
                        return math.hypot(dx, dy)
                    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
                    cx = ax + t * vx
                    cy = ay + t * vy
                    return math.hypot(px - cx, py - cy)

                # Helper to publish open area result
                def publish_open(idx: int, px: float, py: float, yaw_center: float, is_estimated: bool = False) -> None:
                    from std_msgs.msg import Int32
                    from geometry_msgs.msg import PoseStamped
                    # Publish index
                    m = Int32(); m.data = int(idx); self.pub_open_idx.publish(m)
                    # Build pose at center of segment offset
                    # Normal pick near base_link -y: choose normal with negative projection onto base_link y-axis
                    # Compute base_link yaw to map
                    bl_tf = self._lookup_map_to_bl()
                    byaw = bl_tf[2] if bl_tf is not None else 0.0
                    nx, ny = -uy, ux  # normal to the right of axis
                    # Choose normal closer to -base_link y
                    # base_link -y in map frame
                    nby = -math.cos(byaw), -math.sin(byaw)
                    # dot product with (nx,ny) vs (-nx,-ny)
                    dot1 = nx * nby[0] + ny * nby[1]
                    if dot1 > 0.0:
                        nx, ny = -nx, -ny
                    cx = 0.5 * (look_start[0] + look_end[0])
                    cy = 0.5 * (look_start[1] + look_end[1])
                    tx = cx + off_y * nx
                    ty = cy + off_y * ny
                    # shift along line: exact → towards reference by off_x,
                    # estimated → away from reference by off_x_est
                    sign_to_ref = -1.0 if (tend > tref) else 1.0
                    if is_estimated:
                        tx += (-sign_to_ref) * off_x_est * ux
                        ty += (-sign_to_ref) * off_x_est * uy
                    else:
                        tx += sign_to_ref * off_x * ux
                        ty += sign_to_ref * off_x * uy
                    # Pose yaw faces opposite of reference direction (i.e., -yaw_line)
                    pyaw = yaw_line + math.pi
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = now_msg
                    pose.pose.position.x = float(tx)
                    pose.pose.position.y = float(ty)
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.z = float(math.sin(0.5 * pyaw))
                    pose.pose.orientation.w = float(math.cos(0.5 * pyaw))
                    self.pub_parking_pose.publish(pose)
                    # Pose marker disabled

                # Stage 2 & 3: as cluster grows
                # determine outward direction from end (away from reference)
                sign_out = 1.0 if (tend > tref) else -1.0
                dirx = sign_out * ux
                diry = sign_out * uy
                # determine end look segment for block checking and forward search (outward only)
                look_start = (end_x, end_y)
                look_end = (end_x + clear_d * dirx, end_y + clear_d * diry)
                # Search for next cone beyond clear_d along the line direction
                found = None  # (cid, px, py)
                for cid, (px, py) in pts.items():
                    tpt = dirx * (px - end_x) + diry * (py - end_y)
                    if tpt >= clear_d:
                        # ignore cones closer than connect distance from the current end
                        if math.hypot(px - end_x, py - end_y) < connect:
                            continue
                        latp = abs(-(uy) * (px - center_x) + (ux) * (py - center_y))
                        if latp <= lat_tol:
                            found = (cid, px, py)
                            break

                # Stage 2: open_min_length_m <= length <= area2_length_m
                if (length >= open_min) and (length <= area2_len):
                    log_once('stage2_start', '구역 1 탐색 종료, 구역 2 탐색 시작')
                    if found is not None:
                        cone_id = int(found[0])
                        log_once('open_idx_1', f'Open Area idx: 1 (cone_id={cone_id})')
                        look_start = (end_x, end_y)
                        look_end = (found[1], found[2])
                        publish_open(1, end_x, end_y, yaw_line)
                        self._area_decided = True
                        return
                    # Estimated 1 (2-2): until next cone appears
                    log_once('open_idx_1_est', 'Open Area (Estimated) idx: 1')
                    look_start = (end_x, end_y)
                    # If there is a new cone added within (0, clear_d), use it to refine estimate
                    near_best = None
                    best_t = 0.0
                    for cid, (px, py) in pts.items():
                        tpt = dirx * (px - end_x) + diry * (py - end_y)
                        if tpt > 0.0 and tpt < clear_d:
                            if math.hypot(px - end_x, py - end_y) < connect:
                                continue
                            latp = abs(-(uy) * (px - center_x) + (ux) * (py - center_y))
                            if latp <= lat_tol and tpt > best_t:
                                best_t = tpt
                                near_best = (px, py)
                    if near_best is not None:
                        look_end = (near_best[0], near_best[1])
                    else:
                        look_end = (end_x + off_x_est * dirx, end_y + off_x_est * diry)
                    publish_open(1, end_x, end_y, yaw_line, is_estimated=True)
                    return

                # Stage 3: area2_length_m < length < max_area_length_m
                if (length > area2_len) and (length < max_len):
                    # exact idx 2
                    if found is not None:
                        cone_id = int(found[0])
                        log_once('stage3_start', '구역 2 탐색 종료, 구역 3 탐색 시작')
                        log_once('open_idx_2', f'Open Area idx: 2 (cone_id={cone_id})')
                        look_start = (end_x, end_y)
                        look_end = (found[1], found[2])
                        publish_open(2, end_x, end_y, yaw_line)
                        self._area_decided = True
                        return
                    # estimated idx 2 when no further cone
                    log_once('stage3_start', '구역 2 탐색 종료, 구역 3 탐색 시작')
                    log_once('open_idx_2_est', 'Open Area (Estimated) idx: 2')
                    look_start = (end_x, end_y)
                    near_best = None
                    best_t = 0.0
                    for cid, (px, py) in pts.items():
                        tpt = dirx * (px - end_x) + diry * (py - end_y)
                        if tpt > 0.0 and tpt < clear_d:
                            if math.hypot(px - end_x, py - end_y) < connect:
                                continue
                            latp = abs(-(uy) * (px - center_x) + (ux) * (py - center_y))
                            if latp <= lat_tol and tpt > best_t:
                                best_t = tpt
                                near_best = (px, py)
                    if near_best is not None:
                        look_end = (near_best[0], near_best[1])
                    else:
                        look_end = (end_x + off_x_est * dirx, end_y + off_x_est * diry)
                    publish_open(2, end_x, end_y, yaw_line, is_estimated=True)
                    return

                # Stage 4: too long
                if length >= max_len:
                    log_once('error_too_long', '[ERROR] Area Detection Failed : Cluster too long', level='error')
                    self._area_decided = True
                    return
        except Exception as e:
            self.get_logger().warn(f'open-area check failed: {e}')

        self._roi_prev_ids = current_ids


def main() -> None:
    rclpy.init()
    node = ParkingAreaDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
