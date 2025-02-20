#! /usr/bin/env python3

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Vector3
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

class AutocarTF(Node):
    def __init__(self):
        super().__init__('autocar_tf')
        
        self.tf_br_world_to_map = StaticTransformBroadcaster(self)
        self.tf_br_map_to_base_link = TransformBroadcaster(self)

        self.local_origin_sub = self.create_subscription(NavSatFix, '/ublox_gps', self.callback_local_origin, 10)
        self.global_location_sub = self.create_subscription(State2D, '/autocar/state2D', self.callback_global_location, 10)
        
        self.flag_world_to_map = False

        self.get_logger().info("Autocar TF node has been started.")

    def callback_local_origin(self, local_origin):
        if not self.flag_world_to_map:
            lat = local_origin.latitude
            lon = local_origin.longitude
            world_x, world_y = latlon_to_utm(lat, lon)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "map"
            t.transform.translation = Vector3(x=float(world_x), y=float(world_y), z=0.0)
            t.transform.rotation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)  # 기본 회전 설정
            
            self.tf_br_world_to_map.sendTransform(t)
            self.flag_world_to_map = True

    def callback_global_location(self, global_location_msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  
        t.child_frame_id = "base_link"

        t.transform.translation = Vector3(
            x=global_location_msg.pose.x,
            y=global_location_msg.pose.y,
            z=0.0  
        )
        t.transform.rotation = quaternion_from_euler(0, 0, global_location_msg.pose.theta)

        self.tf_br_map_to_base_link.sendTransform(t)

        self.get_logger().info(f"Global location: x={global_location_msg.pose.x}, y={global_location_msg.pose.y}, theta={global_location_msg.pose.theta}")

def main(args=None):
    rclpy.init(args=args)
    node = AutocarTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
