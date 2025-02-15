#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyproj
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point, PoseArray, Vector3
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener, Buffer
import tf_transformations

# 위도, 경도를 UTM 좌표로 변환하는 함수
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

# 쿼터니언을 오일러 각(roll, pitch, yaw)으로 변환하는 함수
def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# 로컬 좌표계를 글로벌 좌표계로 변환하는 함수
def transform_local_to_global(position, orientation, local_point):
    rot_matrix = tf_transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    global_xy = np.dot(rot_matrix[:2, :2], local_point[:2]) + [position.x, position.y]
    global_z = local_point[2] + position.z

    return np.array([global_xy[0], global_xy[1], global_z])

# ROS2 노드 정의
class AutocarTF(Node):
    def __init__(self):
        super().__init__('autocar_tf')

        # 좌표 변환을 위한 오프셋 회전 (yaw -1.444064°)
        self.rot_offset = tf_transformations.quaternion_from_euler(0, 0, np.radians(-1.4440643432812905))
        self.rot_offset_quat = Quaternion(x=self.rot_offset[0], y=self.rot_offset[1], z=self.rot_offset[2], w=self.rot_offset[3])

        # TF 브로드캐스터 (정적, 동적)
        self.tf_br_world_to_map = StaticTransformBroadcaster(self)
        self.tf_br_world_to_base_link = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS2 토픽 구독
        self.local_origin_sub = self.create_subscription(PoseStamped, '/local_xy_origin', self.callback_local_origin, 10)
        self.global_location_sub = self.create_subscription(Odometry, '/location', self.callback_global_location, 10)

        # 맵 좌표계를 설정했는지 확인하는 플래그
        self.flag_world_to_map = False

    # /local_xy_origin 콜백 함수 (맵 좌표계 설정)
    def callback_local_origin(self, local_origin):
        if not self.flag_world_to_map:
            # PoseStamped 메시지에서 위도, 경도 값 추출
            lat = local_origin.pose.position.y
            lon = local_origin.pose.position.x
            world_x, world_y = latlon_to_utm(lat, lon)

            # world → map 변환 정의
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "map"
            t.transform.translation = Vector3(x=float(world_x), y=float(world_y), z=0.0)
            t.transform.rotation = self.rot_offset_quat

            # 변환 정보 브로드캐스트
            self.tf_br_world_to_map.sendTransform(t)
            self.flag_world_to_map = True  # 한 번만 실행되도록 플래그 설정

    # /location 콜백 함수 (차량 위치 정보 갱신)
    def callback_global_location(self, global_location_msg):
        self.location = {
            "position": global_location_msg.pose.pose.position,
            "orientation": global_location_msg.pose.pose.orientation
        }

        # world → base_link 변환 정의
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation = Vector3(
            x=global_location_msg.pose.pose.position.x,
            y=global_location_msg.pose.pose.position.y,
            z=global_location_msg.pose.pose.position.z
        )
        t.transform.rotation = global_location_msg.pose.pose.orientation

        # 변환 정보 브로드캐스트
        self.tf_br_world_to_base_link.sendTransform(t)

# ROS2 노드 실행
def main(args=None):
    rclpy.init(args=args)
    node = AutocarTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
