#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from std_msgs.msg import Bool, Int64
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from planning_msgs.msg import ModeState
from visualization_msgs.msg import MarkerArray, Marker


def wrap_to_pi(angle: float) -> float:
    """각도를 [-π, π] 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    """쿼터니언에서 yaw 각도 추출"""
    return math.atan2(2.0 * q.z * q.w, 1.0 - 2.0 * q.z * q.z)


def yaw_to_quaternion(yaw: float):
    """yaw 각도를 쿼터니언으로 변환"""
    from geometry_msgs.msg import Quaternion
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


class UturnPlanner(Node):
    """
    U턴 플래너 클래스
    
    라바콘 인식 기반으로 U턴 구역에서 자동으로 U턴 경로를 생성하고 실행합니다.
    
    입력 토픽:
    - /sensor_fusion/tracked_rubber_cones (MarkerArray): 라바콘 3D 위치
    - /current_lanelet_id (Int64): 현재 Lanelet ID
    - /mode_state (ModeState): 현재 모드 상태
    - /autocar/location (Odometry): 차량 현재 위치 (UTM 좌표계)
    
    출력 토픽:
    - /uturn_start_flag (Bool): U턴 모드 시작 플래그
    - /uturn_complete_flag (Bool): U턴 모드 완료 플래그
    - /waypoints (Path): U턴 경로
    - /waypoints_points (PoseArray): U턴 웨이포인트
    
    동작 설명:
    1. U턴 구역 진입 및 라바콘 3개 이상 감지 시 U턴 시작
    2. 차량 위치 기준 오른쪽으로 지름 4m 원호 경로 생성
    3. 경로 마지막 점으로부터 2m 이내 도달 시 U턴 완료
    """
    
    def __init__(self):
        super().__init__('uturn_planner')

        # 파라미터 선언
        self.declare_parameter('path_resolution', 0.3)  # 경로 해상도 [m]
        self.declare_parameter('arc_radius', 4.0)  # 원호 반지름 [m] (지름 8m)
        self.declare_parameter('completion_distance', 2.0)  # 완료 거리 [m]
        self.declare_parameter('min_cone_count', 3)  # 최소 라바콘 개수
        self.declare_parameter('min_distance_threshold', 2.5)  # 라바콘 최소 거리 임계치 [m]
        self.declare_parameter('pre_straight_distance', 2.0)  # 원호 전 직진 거리 [m]
        self.declare_parameter('enable_debug_visualization', False)  # 디버그 시각화 활성화
        self.declare_parameter('frame_id', 'map')  # 기본 좌표계
        
        # U턴 구역 ID (selector.py와 동일)
        self.declare_parameter('kcity_uturn_zones', [5, 6, 7])  # K-City U턴 구역
        self.declare_parameter('mirae_uturn_zones', [39])  # 미래관 U턴 구역
        self.declare_parameter('map_type', 'kcity')  # 맵 타입
        
        # 테스트 모드 파라미터
        self.declare_parameter('test_mode_enabled', False)  # 테스트 모드 활성화
        self.declare_parameter('test_vehicle_x', 0.0)  # 테스트용 차량 X 위치 (map 프레임) [m]
        self.declare_parameter('test_vehicle_y', 0.0)  # 테스트용 차량 Y 위치 (map 프레임) [m]
        self.declare_parameter('test_vehicle_yaw_deg', 0.0)  # 테스트용 차량 방향 [도]
        
        # QoS 프로파일 설정
        qos_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_default = QoSProfile(depth=10)
        
        # Publishers
        self.uturn_start_pub = self.create_publisher(
            Bool, '/uturn_start_flag', qos_transient_local)
        self.uturn_complete_pub = self.create_publisher(
            Bool, '/uturn_complete_flag', qos_transient_local)
        self.waypoints_pub = self.create_publisher(
            Path, '/waypoints', qos_default)
        self.waypoints_points_pub = self.create_publisher(
            PoseArray, '/waypoints_points', qos_default)
        
        # 테스트 모드용 시각화 퍼블리셔
        self.test_vehicle_marker_pub = self.create_publisher(
            Marker, '/uturn/test_vehicle_marker', qos_default)
        
        # 디버그 시각화 퍼블리셔
        self.debug_cone_distances_pub = self.create_publisher(
            MarkerArray, '/uturn/debug_cone_distances', qos_default)
            
        # Subscribers
        self.cones_sub = self.create_subscription(
            MarkerArray, '/sensor_fusion/tracked_rubber_cones', self._on_cones, 10)
        self.lanelet_sub = self.create_subscription(
            Int64, '/current_lanelet_id', self._on_lanelet_id, 10)
        self.mode_state_sub = self.create_subscription(
            ModeState, '/mode_state', self._on_mode_state, 10)
        self.location_sub = self.create_subscription(
            Odometry, '/autocar/location', self._on_location, 10)
            
        # TF 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 플래너 상태
        self._is_active = False
        self._current_mode = None
        self._current_lanelet_id = None
        self._cone_count = 0
        self._tracked_cones = []  # 감지된 라바콘 위치 리스트 (map 프레임)
        self._cone_distances = []  # 각 라바콘과 차량 간 거리 리스트 (디버그용)
        self._vehicle_location = None  # UTM 좌표계
        self._start_location = None  # U턴 시작 시점의 위치 (map 프레임)
        self._uturn_path = None  # 생성된 U턴 경로
        self._uturn_started = False  # U턴 시작 플래그
        self._path_published = False  # 경로 발행 완료 플래그
        
        # 테스트 모드 상태
        self._test_mode_published = False  # 테스트 모드 경로 발행 완료 플래그
        self._test_vehicle_pose = None  # 테스트용 차량 위치
        
        # U턴 구역 설정
        map_type = str(self.get_parameter('map_type').value)
        if map_type == 'kcity':
            self._uturn_zone_ids = list(self.get_parameter('kcity_uturn_zones').value)
        else:  # mirae
            self._uturn_zone_ids = list(self.get_parameter('mirae_uturn_zones').value)
        
        # 타이머 설정 (테스트 모드 체크용)
        self.create_timer(0.1, self._on_timer)  # 0.1초마다 테스트 모드 체크
        
        # 로거 설정
        self.get_logger().info('U턴 플래너가 초기화되었습니다.')
        self.get_logger().info(f'U턴 구역 ID: {self._uturn_zone_ids}')
    
    def _is_in_uturn_zone(self) -> bool:
        """현재 U턴 구역에 있는지 확인"""
        return (self._current_lanelet_id is not None and 
                self._current_lanelet_id in self._uturn_zone_ids)
    
    def _on_cones(self, msg: MarkerArray) -> None:
        """라바콘 메시지 콜백 함수"""
        # 테스트 모드에서는 라바콘 처리 스킵
        if bool(self.get_parameter('test_mode_enabled').value):
            return
            
        if not self._is_active:
            return
            
        # 라바콘 개수 및 위치 저장
        cone_count = 0
        self._tracked_cones = []  # 라바콘 위치 리스트 초기화
        
        for marker in msg.markers:
            if marker.type == Marker.CYLINDER or marker.type == Marker.SPHERE:
                cone_count += 1
                # 라바콘 위치를 map 프레임으로 변환하여 저장
                cone_pose = self._transform_cone_to_map(marker)
                if cone_pose is not None:
                    self._tracked_cones.append(cone_pose)
        
        self._cone_count = cone_count
        self.get_logger().debug(f'감지된 라바콘 개수: {cone_count} (map 변환 성공: {len(self._tracked_cones)})')
        
        # U턴 시작 조건 체크
        self._check_uturn_start_condition()
    
    def _on_lanelet_id(self, msg: Int64) -> None:
        """Lanelet ID 메시지 콜백 함수"""
        # 테스트 모드에서는 Lanelet ID 처리 스킵
        if bool(self.get_parameter('test_mode_enabled').value):
            return
            
        self._current_lanelet_id = int(msg.data)
        self.get_logger().debug(f'현재 Lanelet ID: {self._current_lanelet_id}')
        
        # U턴 구역 진입/이탈 체크
        if self._is_in_uturn_zone():
            self.get_logger().info(f'U턴 구역 진입: {self._current_lanelet_id}')
        elif self._uturn_started and not self._is_in_uturn_zone():
            self.get_logger().info('U턴 구역 이탈')
    
    def _on_mode_state(self, msg: ModeState) -> None:
        """모드 상태 메시지 콜백 함수"""
        self._current_mode = msg.current_mode
        
        # 테스트 모드에서는 모드 상태 처리 스킵
        if bool(self.get_parameter('test_mode_enabled').value):
            return
        
        # U턴 모드 활성화 체크
        if msg.current_mode == ModeState.UTURN:
            self._is_active = True
            self.get_logger().info('U턴 모드 활성화')
        else:
            self._is_active = False
            if self._uturn_started:
                self.get_logger().info('U턴 모드 비활성화')
    
    def _on_location(self, msg: Odometry) -> None:
        """차량 위치 메시지 콜백 함수"""
        self._vehicle_location = msg
        
        # 테스트 모드에서는 완료 조건 체크 스킵
        if bool(self.get_parameter('test_mode_enabled').value):
            return
        
        # U턴 완료 조건 체크
        if self._uturn_started and self._path_published:
            self._check_uturn_completion()
    
    def _check_uturn_start_condition(self) -> None:
        """U턴 시작 조건 체크"""
        if self._uturn_started:
            return  # 이미 시작됨
            
        if not self._is_active:
            return  # U턴 모드가 아님
            
        if not self._is_in_uturn_zone():
            return  # U턴 구역이 아님
            
        min_cone_count = int(self.get_parameter('min_cone_count').value)
        if self._cone_count < min_cone_count:
            return  # 라바콘 개수 부족
            
        if self._vehicle_location is None:
            return  # 차량 위치 정보 없음
        
        # 라바콘과 차량 간 최소 거리 계산
        if len(self._tracked_cones) == 0:
            return  # 변환된 라바콘 위치 없음
        
        current_pose = self._transform_utm_to_map(self._vehicle_location)
        if current_pose is None:
            return  # 차량 위치 변환 실패
        
        self._cone_distances = []
        
        for cone_pose in self._tracked_cones:
            distance = math.sqrt(
                (current_pose.pose.position.x - cone_pose.pose.position.x)**2 +
                (current_pose.pose.position.y - cone_pose.pose.position.y)**2
            )
            self._cone_distances.append(distance)
        
        # 최소 거리 계산
        min_distance = min(self._cone_distances)
        min_distance_threshold = float(self.get_parameter('min_distance_threshold').value)
        
        # 디버그 로그 (debug 레벨)
        self.get_logger().debug(
            f'라바콘 최소 거리: {min_distance:.2f}m (임계치: {min_distance_threshold:.2f}m)'
        )
        
        if min_distance > min_distance_threshold:
            return  # 가장 가까운 라바콘도 임계치보다 멀면 시작하지 않음
        
        # 디버그 시각화 발행
        if bool(self.get_parameter('enable_debug_visualization').value):
            self._publish_debug_visualization(current_pose)
        
        # U턴 시작!
        self._start_uturn()
    
    def _start_uturn(self) -> None:
        """U턴 시작 처리"""
        self._uturn_started = True
        
        # 시작 플래그 발행
        start_msg = Bool()
        start_msg.data = True
        self.uturn_start_pub.publish(start_msg)
        
        # 시작 시점의 차량 위치를 map 프레임으로 변환하여 저장
        self._start_location = self._transform_utm_to_map(self._vehicle_location)
        if self._start_location is None:
            self.get_logger().error('UTM → Map 변환 실패')
            return
        
        # U턴 경로 생성
        self._generate_uturn_path()
        
        self.get_logger().info('U턴 시작! 경로 생성 및 발행')
    
    def _transform_utm_to_map(self, utm_pose: Odometry) -> Optional[PoseStamped]:
        """UTM 좌표를 map 프레임으로 변환"""
        try:
            # Odometry를 PoseStamped로 변환
            utm_pose_stamped = PoseStamped()
            utm_pose_stamped.header = utm_pose.header
            utm_pose_stamped.header.frame_id = 'world'  # UTM 좌표계
            utm_pose_stamped.pose = utm_pose.pose.pose
            
            # TF2 변환
            map_pose = self.tf_buffer.transform(
                utm_pose_stamped,
                'map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            return map_pose
            
        except Exception as e:
            self.get_logger().error(f"UTM→Map 변환 실패: {e}")
            return None
    
    def _transform_cone_to_map(self, marker: Marker) -> Optional[PoseStamped]:
        """라바콘 마커를 센서 프레임에서 map 프레임으로 변환"""
        try:
            # 마커를 PoseStamped로 변환
            cone_pose_stamped = PoseStamped()
            cone_pose_stamped.header = marker.header
            cone_pose_stamped.header.stamp = rclpy.time.Time().to_msg()  # 최신 TF 사용
            cone_pose_stamped.pose = marker.pose
            
            # TF2 변환
            map_pose = self.tf_buffer.transform(
                cone_pose_stamped,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            return map_pose
            
        except Exception as e:
            self.get_logger().debug(f"라바콘 변환 실패 ({marker.header.frame_id}→map): {e}")
            return None
    
    def _generate_uturn_path(self) -> None:
        """U턴 경로 생성 (직진 + 지름 8m 원호)"""
        if self._start_location is None:
            return
        
        # 파라미터 가져오기
        radius = float(self.get_parameter('arc_radius').value)  # 4.0m
        resolution = float(self.get_parameter('path_resolution').value)  # 0.3m
        straight_distance = float(self.get_parameter('pre_straight_distance').value)  # 2.0m
        
        # 차량의 현재 위치와 방향
        start_x = self._start_location.pose.position.x
        start_y = self._start_location.pose.position.y
        start_yaw = quaternion_to_yaw(self._start_location.pose.orientation)
        
        # 직진 끝점 계산
        straight_end_x = start_x + straight_distance * math.cos(start_yaw)
        straight_end_y = start_y + straight_distance * math.sin(start_yaw)
        
        # 원호 중심점 계산 (직진 끝점에서 오른쪽으로 radius만큼)
        # base_link 기준 y < 0이 오른쪽이므로, map 프레임에서는 +sin(yaw) 방향
        center_x = straight_end_x + radius * math.sin(start_yaw)
        center_y = straight_end_y - radius * math.cos(start_yaw)
        
        # 경로 생성
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # 1. 직진 구간 웨이포인트 생성
        num_straight_points = max(1, int(straight_distance / resolution))
        for i in range(num_straight_points + 1):
            t = i / num_straight_points
            px = start_x + t * (straight_end_x - start_x)
            py = start_y + t * (straight_end_y - start_y)
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(start_yaw)
            path.poses.append(pose)
        
        # 2. 원호 구간 웨이포인트 생성 (180도)
        arc_length = math.pi * radius  # 반원 둘레
        num_arc_points = max(1, int(arc_length / resolution))
        
        for i in range(num_arc_points + 1):
            # 각도 계산 (-π/2에서 π/2까지, 직진 끝점에서 시작)
            angle = -math.pi/2 + math.pi * i / num_arc_points
            
            # 원호 위의 점 계산
            point_x = center_x + radius * math.cos(start_yaw + angle)
            point_y = center_y + radius * math.sin(start_yaw + angle)
            
            # 웨이포인트 생성
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point_x
            pose.pose.position.y = point_y
            pose.pose.position.z = 0.0
            
            # 방향은 원호의 접선 방향
            tangent_yaw = start_yaw + angle + math.pi/2
            pose.pose.orientation = yaw_to_quaternion(tangent_yaw)
            
            path.poses.append(pose)
        
        self._uturn_path = path
        
        # 경로 발행
        self.waypoints_pub.publish(path)
        self._publish_waypoints_points(path)
        
        self._path_published = True
        self.get_logger().info(f'U턴 경로 생성: 직진 {straight_distance:.1f}m + 원호 반지름 {radius:.1f}m')
        self.get_logger().info(f'U턴 경로 생성 완료: {len(path.poses)}개 점 (직진: {num_straight_points+1}개, 원호: {num_arc_points+1}개)')
    
    def _publish_waypoints_points(self, path: Path) -> None:
        """Path를 PoseArray로 변환하여 발행"""
        arr = PoseArray()
        arr.header = path.header
        
        for pose_stamped in path.poses:
            pose = Pose()
            pose.position = pose_stamped.pose.position
            pose.orientation = pose_stamped.pose.orientation
            arr.poses.append(pose)
        
        self.waypoints_points_pub.publish(arr)
    
    def _check_uturn_completion(self) -> None:
        """U턴 완료 조건 체크"""
        if self._uturn_path is None or len(self._uturn_path.poses) == 0:
            return
        
        # 현재 차량 위치를 map 프레임으로 변환
        current_pose = self._transform_utm_to_map(self._vehicle_location)
        if current_pose is None:
            return
        
        # 마지막 웨이포인트와의 거리 계산
        last_waypoint = self._uturn_path.poses[-1]
        distance = math.sqrt(
            (current_pose.pose.position.x - last_waypoint.pose.position.x)**2 +
            (current_pose.pose.position.y - last_waypoint.pose.position.y)**2
        )
        
        completion_distance = float(self.get_parameter('completion_distance').value)
        
        if distance <= completion_distance:
            # U턴 완료!
            self._complete_uturn()
    
    def _complete_uturn(self) -> None:
        """U턴 완료 처리"""
        # 완료 플래그 발행
        complete_msg = Bool()
        complete_msg.data = True
        self.uturn_complete_pub.publish(complete_msg)
        
        # 상태 초기화
        self._uturn_started = False
        self._path_published = False
        self._uturn_path = None
        self._start_location = None
        
        self.get_logger().info('U턴 완료!')
    
    def _publish_debug_visualization(self, vehicle_pose: PoseStamped) -> None:
        """라바콘과 차량을 잇는 선을 시각화"""
        if len(self._tracked_cones) == 0 or len(self._cone_distances) == 0:
            return
        
        marker_array = MarkerArray()
        
        # 라바콘과 차량을 잇는 선 마커
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'uturn_debug'
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        
        # 선의 두께
        line_marker.scale.x = 0.1  # 선 두께
        
        # 선 색상 (노란색)
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8
        
        # 차량 위치와 각 라바콘을 잇는 선 생성
        vehicle_point = Point()
        vehicle_point.x = vehicle_pose.pose.position.x
        vehicle_point.y = vehicle_pose.pose.position.y
        vehicle_point.z = 0.0
        
        for i, cone_pose in enumerate(self._tracked_cones):
            # 차량 위치
            line_marker.points.append(vehicle_point)
            
            # 라바콘 위치
            cone_point = Point()
            cone_point.x = cone_pose.pose.position.x
            cone_point.y = cone_pose.pose.position.y
            cone_point.z = 0.0
            line_marker.points.append(cone_point)
        
        marker_array.markers.append(line_marker)
        
        # 거리 텍스트 마커 (선택적)
        for i, (cone_pose, distance) in enumerate(zip(self._tracked_cones, self._cone_distances)):
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'uturn_debug'
            text_marker.id = i + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 텍스트 위치 (라바콘과 차량 중간점)
            text_marker.pose.position.x = (vehicle_pose.pose.position.x + cone_pose.pose.position.x) / 2
            text_marker.pose.position.y = (vehicle_pose.pose.position.y + cone_pose.pose.position.y) / 2
            text_marker.pose.position.z = 0.5
            text_marker.pose.orientation.w = 1.0
            
            # 텍스트 크기
            text_marker.scale.z = 0.3
            
            # 텍스트 색상 (흰색)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # 텍스트 내용
            text_marker.text = f'{distance:.1f}m'
            
            marker_array.markers.append(text_marker)
        
        # 마커 발행
        self.debug_cone_distances_pub.publish(marker_array)
    
    # ==================== 테스트 모드 함수 ====================
    def _create_test_vehicle_pose(self) -> PoseStamped:
        """테스트용 차량 위치 생성 (map 프레임)"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter('test_vehicle_x').value)
        pose.pose.position.y = float(self.get_parameter('test_vehicle_y').value)
        pose.pose.position.z = 0.0
        yaw_deg = float(self.get_parameter('test_vehicle_yaw_deg').value)
        pose.pose.orientation = yaw_to_quaternion(math.radians(yaw_deg))
        return pose

    def _run_test_mode(self) -> None:
        """테스트 모드: 파라미터 기반 U턴 경로 생성 (한 번만 실행)"""
        
        # 한 번만 실행
        if self._test_mode_published:
            return
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("U턴 테스트 모드 시작")
        self.get_logger().info("=" * 60)
        
        # 1. 테스트 데이터 생성
        test_vehicle = self._create_test_vehicle_pose()
        self._test_vehicle_pose = test_vehicle  # 시각화용으로 저장
        
        self.get_logger().info(f"[TEST] 차량 위치: ({test_vehicle.pose.position.x:.2f}, {test_vehicle.pose.position.y:.2f})")
        self.get_logger().info(f"[TEST] 차량 방향: {math.degrees(quaternion_to_yaw(test_vehicle.pose.orientation)):.1f}°")
        
        # 2. U턴 경로 생성
        self.get_logger().info("-" * 60)
        self.get_logger().info("[TEST] U턴 경로 생성 중...")
        try:
            # 시작 위치 설정
            self._start_location = test_vehicle
            
            # U턴 경로 생성
            self._generate_uturn_path()
            
            if self._uturn_path is not None:
                self.get_logger().info(f"[TEST] U턴 경로 생성 완료: {len(self._uturn_path.poses)}개 점")
                
                # 시작 플래그 발행
                start_msg = Bool()
                start_msg.data = True
                self.uturn_start_pub.publish(start_msg)
                self.get_logger().info("[TEST] U턴 시작 플래그 발행")
                
                # 경로 발행
                self.waypoints_pub.publish(self._uturn_path)
                self._publish_waypoints_points(self._uturn_path)
                self.get_logger().info("[TEST] U턴 경로 발행 완료")
                
                # 경로 정보 출력
                self._print_path_info()
                
            else:
                self.get_logger().error("[TEST] U턴 경로 생성 실패")
        except Exception as e:
            self.get_logger().error(f"[TEST] U턴 경로 생성 오류: {e}")
            import traceback
            self.get_logger().error(f"[TEST] 상세: {traceback.format_exc()}")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("[TEST] U턴 테스트 모드 완료")
        self.get_logger().info("[TEST] RViz2에서 /waypoints 토픽으로 경로 확인 가능")
        self.get_logger().info("=" * 60)
        
        # 한 번만 실행
        self._test_mode_published = True
    
    def _print_path_info(self) -> None:
        """생성된 경로 정보 출력"""
        if self._uturn_path is None or len(self._uturn_path.poses) == 0:
            return
        
        self.get_logger().info("-" * 60)
        self.get_logger().info("[TEST] 경로 상세 정보:")
        
        # 시작점과 끝점 정보
        start_pose = self._uturn_path.poses[0]
        end_pose = self._uturn_path.poses[-1]
        
        self.get_logger().info(f"[TEST] 시작점: ({start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f})")
        self.get_logger().info(f"[TEST] 끝점: ({end_pose.pose.position.x:.2f}, {end_pose.pose.position.y:.2f})")
        
        # 직진 구간 정보
        straight_distance = float(self.get_parameter('pre_straight_distance').value)
        start_yaw = quaternion_to_yaw(start_pose.pose.orientation)
        straight_end_x = start_pose.pose.position.x + straight_distance * math.cos(start_yaw)
        straight_end_y = start_pose.pose.position.y + straight_distance * math.sin(start_yaw)
        
        self.get_logger().info(f"[TEST] 직진 구간: {straight_distance:.1f}m")
        self.get_logger().info(f"[TEST] 직진 끝점: ({straight_end_x:.2f}, {straight_end_y:.2f})")
        
        # 원호 중심점 계산
        radius = float(self.get_parameter('arc_radius').value)
        center_x = straight_end_x + radius * math.sin(start_yaw)
        center_y = straight_end_y - radius * math.cos(start_yaw)
        
        self.get_logger().info(f"[TEST] 원호 중심: ({center_x:.2f}, {center_y:.2f})")
        self.get_logger().info(f"[TEST] 원호 반지름: {radius:.2f}m (지름: {radius*2:.2f}m)")
        
        # 경로 길이 계산
        total_length = 0.0
        for i in range(len(self._uturn_path.poses) - 1):
            p1 = self._uturn_path.poses[i]
            p2 = self._uturn_path.poses[i + 1]
            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y
            total_length += math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(f"[TEST] 경로 총 길이: {total_length:.2f}m")
        self.get_logger().info(f"[TEST] 웨이포인트 간격: {float(self.get_parameter('path_resolution').value):.2f}m")
        self.get_logger().info("-" * 60)
    
    def _publish_test_vehicle_marker(self) -> None:
        """테스트 모드에서 차량 위치와 방향을 시각화"""
        if self._test_vehicle_pose is None:
            return
        
        # 차량 본체 마커 (사각형)
        vehicle_marker = Marker()
        vehicle_marker.header.frame_id = 'map'
        vehicle_marker.header.stamp = self.get_clock().now().to_msg()
        vehicle_marker.ns = 'uturn_test_vehicle'
        vehicle_marker.id = 0
        vehicle_marker.type = Marker.CUBE
        vehicle_marker.action = Marker.ADD
        
        # 차량 크기 (주차 플래너와 동일)
        vehicle_marker.scale.x = 2.02  # 차량 길이 [m]
        vehicle_marker.scale.y = 1.16  # 차량 폭 [m]
        vehicle_marker.scale.z = 0.5   # 차량 높이 [m]
        
        # 차량 위치와 방향
        vehicle_marker.pose = self._test_vehicle_pose.pose
        vehicle_marker.pose.position.z = 0.25  # 바닥에서 약간 떠있게
        
        # 차량 색상 (파란색)
        vehicle_marker.color.r = 0.0
        vehicle_marker.color.g = 0.0
        vehicle_marker.color.b = 1.0
        vehicle_marker.color.a = 0.8
        
        # 방향 화살표 마커
        arrow_marker = Marker()
        arrow_marker.header = vehicle_marker.header
        arrow_marker.ns = 'uturn_test_vehicle'
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # 화살표 크기
        arrow_marker.scale.x = 2.0  # 화살표 길이
        arrow_marker.scale.y = 0.2  # 화살표 폭
        arrow_marker.scale.z = 0.2  # 화살표 높이
        
        # 화살표 위치와 방향
        arrow_marker.pose = self._test_vehicle_pose.pose
        arrow_marker.pose.position.z = 0.6  # 차량 위에 표시
        
        # 화살표 색상 (빨간색)
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        
        # 마커 발행
        self.test_vehicle_marker_pub.publish(vehicle_marker)
        self.test_vehicle_marker_pub.publish(arrow_marker)
    
    def _on_timer(self) -> None:
        """주기적으로 호출되는 메인 루프 (테스트 모드 체크)"""
        # 테스트 모드 분기
        if bool(self.get_parameter('test_mode_enabled').value):
            self._run_test_mode()
            # 테스트 모드에서 차량 마커 시각화
            if self._test_vehicle_pose is not None:
                self._publish_test_vehicle_marker()


def main(args=None):
    """U턴 플래너 노드의 메인 함수"""
    rclpy.init(args=args)
    
    try:
        node = UturnPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
