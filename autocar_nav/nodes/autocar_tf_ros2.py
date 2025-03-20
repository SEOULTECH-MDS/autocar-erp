#! /usr/bin/env python3

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Vector3
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener, Buffer
from autocar_nav.euler_from_quaternion import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# 위도, 경도를 UTM 좌표로 변환하는 함수
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

class AutocarTF(Node):
    def __init__(self):
        super().__init__('autocar_tf')
        
        # QoS 설정
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # TF 브로드캐스터 설정
        self.tf_br_static = StaticTransformBroadcaster(self)
        self.tf_br_dynamic = TransformBroadcaster(self)
        
        # 파라미터 설정
        self.declare_parameter('map_origin_lat', 0.0)
        self.declare_parameter('map_origin_lon', 0.0)
        
        # 구독
        self.global_location_sub = self.create_subscription(
            Odometry, 
            '/autocar/location', 
            self.callback_global_location, 
            qos)
        
        # map 프레임 초기화
        self.setup_map_frame()
        
    def setup_map_frame(self):
        try:
            # 설정된 map 원점 가져오기
            lat = self.get_parameter('map_origin_lat').value
            lon = self.get_parameter('map_origin_lon').value
            
            # UTM 변환
            world_x, world_y = latlon_to_utm(lat, lon)
            
            # map 원점 좌표 저장
            self.map_origin_x = world_x
            self.map_origin_y = world_y
            
            # world -> map 변환 설정
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "map"
            
            t.transform.translation = Vector3(x=float(world_x), y=float(world_y), z=0.0)
            t.transform.rotation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            
            self.tf_br_static.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Map frame setup failed: {str(e)}')

    def callback_global_location(self, msg):
        try:
            # map -> base_link 변환
            t = TransformStamped()
            t.header.stamp = msg.header.stamp  # 메시지의 타임스탬프 사용
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            
            # world 좌표를 map 좌표로 변환
            map_x = msg.pose.pose.position.x - self.map_origin_x
            map_y = msg.pose.pose.position.y - self.map_origin_y
            
            t.transform.translation = Vector3(x=map_x, y=map_y, z=0.0)
            t.transform.rotation = msg.pose.pose.orientation
            
            self.tf_br_dynamic.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Transform broadcast failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AutocarTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
