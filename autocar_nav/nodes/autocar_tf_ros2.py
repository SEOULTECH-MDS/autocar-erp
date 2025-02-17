#! /usr/bin/env python3

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point, Vector3
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from autocar_msgs.msg import State2D 


from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener, Buffer

# 위도, 경도를 UTM 좌표로 변환하는 함수
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

# 오일러 각을 쿼터니언으로 변환하는 함수
def quaternion_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

# 로컬 좌표계를 글로벌 좌표계로 변환하는 함수
def transform_local_to_global(position, orientation, local_point):
    yaw = 2 * np.arctan2(orientation.z, orientation.w)  # Yaw 추출
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ])
    global_xy = np.dot(rotation_matrix, local_point[:2]) + [position.x, position.y]
    global_z = local_point[2] + position.z
    return np.array([global_xy[0], global_xy[1], global_z])

class AutocarTF(Node):
    def __init__(self):
        super().__init__('autocar_tf')
        
        # 오프셋 회전 (yaw -1.444064°)
        self.rot_offset_quat = quaternion_from_euler(0, 0, np.radians(-1.4440643432812905))

        # TF 브로드캐스터
        self.tf_br_world_to_map = StaticTransformBroadcaster(self)
        self.tf_br_world_to_base_link = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS2 토픽 구독
        # self.local_origin_sub = self.create_subscription(PoseStamped, '/local_xy_origin', self.callback_local_origin, 10)
        # self.global_location_sub = self.create_subscription(Odometry, '/location', self.callback_global_location, 10)
        self.local_origin_sub = self.create_subscription(NavSatFix, '/ublox_gps', self.callback_local_origin, 10)
        self.global_location_sub = self.create_subscription(State2D, '/autocar/state2D', self.callback_global_location, 10)
        
        self.flag_world_to_map = False

    def callback_local_origin(self, local_origin):
        if not self.flag_world_to_map:
            # lat = local_origin.pose.position.y
            # lon = local_origin.pose.position.x
            lat = local_origin.latitude
            lon = local_origin.longitude
            world_x, world_y = latlon_to_utm(lat, lon)
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "map"
            t.transform.translation = Vector3(x=float(world_x), y=float(world_y), z=0.0)
            t.transform.rotation = self.rot_offset_quat
            
            self.tf_br_world_to_map.sendTransform(t)
            self.flag_world_to_map = True

    def callback_global_location(self, global_location_msg):
        self.location = {
            # "position": global_location_msg.pose.pose.position,
            # "orientation": global_location_msg.pose.pose.orientation
            "position": global_location_msg.pose,
            "orientation": global_location_msg.twist
        }

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation = Vector3(
            x=global_location_msg.pose.x,
            y=global_location_msg.pose.y,
            z=0.0  # z는 0으로 설정
        )
        t.transform.rotation = quaternion_from_euler(0, 0, global_location_msg.pose.theta)  # yaw 값을 회전으로 설정

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
