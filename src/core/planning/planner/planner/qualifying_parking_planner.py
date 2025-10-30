#!/usr/bin/env python3
from typing import Dict, List, Optional, Tuple

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

try:
    from pyproj import Transformer
    _HAS_PYPROJ = True
except Exception:
    _HAS_PYPROJ = False

def _normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class QualifyingParkingPlanner(Node):
    """
    예선 주차 전용 실행 노드

    동작 개요:
    - 활성화 Lanelet ID(기본 2)에 진입하면 미리 정의된 OSM의 parking_path(path_id) 경로를 따라 직진
    - 경로 종단 도달 시 3초 정지
    - 이후 지정 거리만큼 후진 경로 발행
    - 완료되면 /parking_complete_flag 발행 및 로컬 경로 발행 중단

    입력:
    - /current_lanelet_id (Int64)
    - /autocar/location (Odometry)

    출력:
    - /waypoints (Path)
    - /reverse_flag (Bool)
    - /parking_stop_flag (Bool)
    - /parking_complete_flag (Bool)
    - /parking/pose_ready (Bool)  # 예선에서도 모드 전환 트리거로 사용 가능
    """

    def __init__(self) -> None:
        super().__init__('qualifying_parking_planner')

        # Parameters
        self.declare_parameter(
            'map_osm_path',
            'src/core/localization/localization_core/data/kcity_qualifying/lanelet2_map.osm',
        )
        # self.declare_parameter(
        #     'map_osm_path',
        #     'src/core/localization/localization_core/data/testing_parking/lanelet2_map.osm',
        # )
        self.declare_parameter('path_id', 1) 
        self.declare_parameter('activation_lanelet_id', 1) # kcity 2
        self.declare_parameter('stop_duration_sec', 3.0)
        self.declare_parameter('reverse_distance_m', 8.0)
        self.declare_parameter('reach_dist_thresh_m', 1.0)
        self.declare_parameter('reach_speed_thresh_mps', 0.05)
        self.declare_parameter('reach_yaw_thresh_deg', 30.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        # Map origin for lat/lon → local map conversion (align with map.launch.py defaults)
        self.declare_parameter('map_origin_lat', 37.239205)
        self.declare_parameter('map_origin_lon', 126.773193)

        

        qos = QoSProfile(depth=10)

        # Publishers
        self.pub_path = self.create_publisher(Path, '/waypoints', qos)
        self.pub_reverse = self.create_publisher(Bool, '/reverse_flag', qos)
        self.pub_stop = self.create_publisher(Bool, '/parking_stop_flag', qos)
        self.pub_complete = self.create_publisher(Bool, '/parking_complete_flag', qos)
        self.pub_pose_ready = self.create_publisher(Bool, '/parking/pose_ready', qos)

        # Subscribers
        self.sub_lanelet = self.create_subscription(Int64, '/current_lanelet_id', self._on_lanelet, qos)
        self.sub_odom = self.create_subscription(Odometry, '/autocar/location', self._on_odom, qos)

        # Internal state
        self._current_lanelet_id: Optional[int] = None
        self._odom: Optional[Odometry] = None

        self._forward_path: Optional[Path] = None
        self._forward_last_yaw: Optional[float] = None
        self._reverse_path: Optional[Path] = None

        self._armed_published: bool = False  # pose_ready 1회성 발행

        self._stage: str = 'IDLE'  # IDLE -> ARMED -> FORWARD -> STOP -> REVERSE -> COMPLETE
        self._stop_start_time_sec: Optional[float] = None

        # Load path upfront
        try:
            self._forward_path, self._forward_last_yaw = self._load_forward_path()
            if self._forward_path is None:
                self.get_logger().error('OSM에서 forward 경로를 불러오지 못했습니다. 노드를 종료합니다.')
        except Exception as exc:
            self.get_logger().error(f'경로 로드 중 예외: {exc}')

        # Timer
        period = 1.0 / max(1e-3, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(period, self._on_timer)

        self.get_logger().info('Qualifying parking planner started.')

    # ---------------------- Subscriptions ----------------------
    def _on_lanelet(self, msg: Int64) -> None:
        self._current_lanelet_id = int(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    # ---------------------- Core timer ----------------------
    def _on_timer(self) -> None:
        if self._forward_path is None:
            return

        activation_id = int(self.get_parameter('activation_lanelet_id').value)

        if self._stage == 'IDLE':
            if self._current_lanelet_id == activation_id:
                self._stage = 'ARMED'
                # pose_ready 트리거 (예선 셀렉터 확장 대비)
                if not self._armed_published:
                    self._publish_pose_ready_once()
                    self._armed_published = True
                self.get_logger().info('Lanelet 진입 감지 - ARMED 상태로 전환')

        elif self._stage == 'ARMED':
            # 즉시 FORWARD 시작
            self._publish_reverse(False)
            self._publish_stop(False)
            self._stage = 'FORWARD'
            self.get_logger().info('FORWARD 단계 시작')

        elif self._stage == 'FORWARD':
            self._publish_path(self._forward_path)
            if self._has_reached_path_end(self._forward_path):
                self._stage = 'STOP'
                self._stop_start_time_sec = self._now_sec()
                self._publish_stop(True)
                self.get_logger().info('경로 종단 도달 - STOP 단계로 전환')

        elif self._stage == 'STOP':
            self._publish_path(self._forward_path)  # 유지
            if self._stop_start_time_sec is not None:
                elapsed = self._now_sec() - self._stop_start_time_sec
                if elapsed >= float(self.get_parameter('stop_duration_sec').value):
                    self._publish_stop(False)
                    # 후진 경로 생성
                    self._reverse_path = self._create_reverse_path()
                    if self._reverse_path is None:
                        self.get_logger().error('후진 경로 생성 실패 - 미션 중단')
                        self._stage = 'COMPLETE'
                    else:
                        self._publish_reverse(True)
                        self._stage = 'REVERSE'
                        self.get_logger().info('STOP 완료 - REVERSE 단계 시작')

        elif self._stage == 'REVERSE':
            if self._reverse_path is not None:
                self._publish_path(self._reverse_path)
                if self._has_reached_path_end(self._reverse_path):
                    self._publish_reverse(False)
                    self._stage = 'COMPLETE'
                    self.get_logger().info('후진 완료 - COMPLETE 단계로 전환')

        elif self._stage == 'COMPLETE':
            self._publish_complete_once()
            # 이후 로컬 경로 발행 중단 (모드 복귀를 기대)
            pass

    # ---------------------- Helpers ----------------------
    def _publish_path(self, path: Path) -> None:
        self.pub_path.publish(path)

    def _publish_reverse(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_reverse.publish(msg)

    def _publish_stop(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_stop.publish(msg)

    def _publish_complete_once(self) -> None:
        msg = Bool()
        msg.data = True
        self.pub_complete.publish(msg)

    def _publish_pose_ready_once(self) -> None:
        msg = Bool()
        msg.data = True
        self.pub_pose_ready.publish(msg)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_forward_path(self) -> Tuple[Optional[Path], Optional[float]]:
        osm_path = str(self.get_parameter('map_osm_path').value)
        target_path_id = str(self.get_parameter('path_id').value)

        tree = ET.parse(osm_path)
        root = tree.getroot()

        # Parse nodes -> prefer (local_x, local_y); fallback to (lat, lon)→map(x,y) using UTM
        node_xy: Dict[str, Tuple[float, float]] = {}
        lat0 = float(self.get_parameter('map_origin_lat').value)
        lon0 = float(self.get_parameter('map_origin_lon').value)
        for node in root.findall('node'):
            node_id = node.get('id')
            if node_id is None:
                continue
            # Prefer local_x/local_y tags inside node
            local_x = None
            local_y = None
            for tag in node.findall('tag'):
                k = tag.get('k') or ''
                v = tag.get('v') or ''
                if k == 'local_x':
                    try:
                        local_x = float(v)
                    except Exception:
                        pass
                elif k == 'local_y':
                    try:
                        local_y = float(v)
                    except Exception:
                        pass
            if local_x is not None and local_y is not None:
                node_xy[node_id] = (local_x, local_y)
                continue

            # Fallback: lat/lon attributes present
            lat_attr = node.get('lat')
            lon_attr = node.get('lon')
            if lat_attr is not None and lon_attr is not None:
                try:
                    lat = float(lat_attr)
                    lon = float(lon_attr)
                    dx, dy = self._latlon_to_xy(lat, lon, lat0, lon0)
                    node_xy[node_id] = (dx, dy)
                except Exception:
                    pass

        # Find way by tags
        way_nodes: List[str] = []
        for way in root.findall('way'):
            tags = { (t.get('k') or ''): (t.get('v') or '') for t in way.findall('tag') }
            if tags.get('type') == 'parking_path' and tags.get('path_id') == target_path_id:
                way_nodes = [nd.get('ref') for nd in way.findall('nd') if nd.get('ref') is not None]
                break

        if not way_nodes:
            self.get_logger().error(f'OSM에서 type=parking_path, path_id={target_path_id} way를 찾지 못했습니다.')
            return None, None

        poses: List[PoseStamped] = []
        for ref in way_nodes:
            if ref not in node_xy:
                self.get_logger().error(f'노드 {ref} 좌표 변환 실패(local_x/local_y 또는 lat/lon 필요).')
                return None, None
            x, y = node_xy[ref]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            poses.append(pose)

        # Compute yaw for poses
        last_yaw = 0.0
        for i in range(len(poses)):
            if i < len(poses) - 1:
                dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
                dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
                yaw = math.atan2(dy, dx)
                last_yaw = yaw
            poses[i].pose.orientation.z = math.sin(last_yaw * 0.5)
            poses[i].pose.orientation.w = math.cos(last_yaw * 0.5)

        path = Path()
        path.header.frame_id = 'map'
        path.poses = poses
        return path, last_yaw

    def _latlon_to_xy(self, lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
        """Convert WGS84 lat/lon to local map (x,y) meters using UTM aligned to map.launch origin.
        If pyproj is unavailable, fallback to meters-per-degree approximation.
        """
        if _HAS_PYPROJ:
            # Determine UTM zone from origin lon (consistent with loader using UTM)
            zone = int(math.floor((lon0 + 180.0) / 6.0) + 1)
            is_north = lat0 >= 0.0
            epsg = 32600 + zone if is_north else 32700 + zone  # 326xx: north, 327xx: south
            transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
            e, n = transformer.transform(lon, lat)
            e0, n0 = transformer.transform(lon0, lat0)
            return e - e0, n - n0
        # Fallback approximation (less accurate)
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
        dx = (lon - lon0) * meters_per_deg_lon
        dy = (lat - lat0) * meters_per_deg_lat
        return dx, dy

    def _latlon_to_utm(self, lat: float, lon: float) -> Tuple[float, float]:
            """Convert WGS84 lat/lon to UTM coordinates (absolute UTM, not relative to origin)."""
            if _HAS_PYPROJ:
                # Determine UTM zone
                zone = int(math.floor((lon + 180.0) / 6.0) + 1)
                is_north = lat >= 0.0
                epsg = 32600 + zone if is_north else 32700 + zone  # 326xx: north, 327xx: south
                transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
                e, n = transformer.transform(lon, lat)
                return e, n
            else:
                # Fallback: use simple approximation (less accurate)
                # This is a very rough approximation and should not be used for precise calculations
                meters_per_deg_lat = 111320.0
                meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat))
                x = lon * meters_per_deg_lon
                y = lat * meters_per_deg_lat
                return x, y


    def _has_reached_path_end(self, path: Path) -> bool:
        if self._odom is None or not path.poses:
            return False
        # Use last pose as goal
        gx = path.poses[-1].pose.position.x
        gy = path.poses[-1].pose.position.y
        
        # Convert odom from UTM to map coordinates by subtracting map origin
        lat0 = float(self.get_parameter('map_origin_lat').value)
        lon0 = float(self.get_parameter('map_origin_lon').value)
        
        # Get current odom position in UTM
        odom_utm_x = self._odom.pose.pose.position.x 
        odom_utm_y = self._odom.pose.pose.position.y
        
        # Convert map origin to UTM coordinates
        origin_utm_x, origin_utm_y = self._latlon_to_utm(lat0, lon0)
        
        # Convert odom to map coordinates
        ox = odom_utm_x - origin_utm_x
        oy = odom_utm_y - origin_utm_y
        
        dist = math.hypot(gx - ox, gy - oy)
        if dist > float(self.get_parameter('reach_dist_thresh_m').value):
            return False
        # Optional speed check
        try:
            speed = abs(self._odom.twist.twist.linear.x)
        except Exception:
            speed = 0.0
        if speed >= float(self.get_parameter('reach_speed_thresh_mps').value):
            return False
        return True

    def _create_reverse_path(self) -> Optional[Path]:
        if self._forward_path is None or not self._forward_path.poses:
            return None
        end_pose = self._forward_path.poses[-1]
        yaw = self._forward_last_yaw if self._forward_last_yaw is not None else 0.0
        reverse_dist = float(self.get_parameter('reverse_distance_m').value)
        step = 0.5
        num_steps = max(1, int(math.ceil(reverse_dist / step)))
        poses: List[PoseStamped] = []
        for i in range(1, num_steps + 1):
            s = min(reverse_dist, i * step)
            x = end_pose.pose.position.x - s * math.cos(yaw)
            y = end_pose.pose.position.y - s * math.sin(yaw)
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            # Orientation: facing backward (yaw + pi)
            byaw = _normalize_angle(yaw + math.pi)
            p.pose.orientation.z = math.sin(byaw * 0.5)
            p.pose.orientation.w = math.cos(byaw * 0.5)
            poses.append(p)

        path = Path()
        path.header.frame_id = 'map'
        path.poses = poses
        return path


def main(args=None):
    rclpy.init(args=args)
    node = QualifyingParkingPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Qualifying parking planner shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


