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

# 위도, 경도를 UTM 좌표로 변환하는 함수
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

class AutocarTF(Node):
    def __init__(self):
        super().__init__('autocar_tf')
        
        self.tf_br_w2m = StaticTransformBroadcaster(self)
        self.tf_br_w2bl = TransformBroadcaster(self)

        self.local_origin_sub = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.callback_local_origin, 10)
        self.global_location_sub = self.create_subscription(Odometry, '/autocar/location', self.callback_global_location, 10)
        
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
            
            self.tf_br_w2m.sendTransform(t)
            self.flag_world_to_map = True

    def callback_global_location(self, global_location_msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  
        t.child_frame_id = "base_link"

        t.transform.translation = Vector3(
            x=global_location_msg.pose.pose.position.x,
            y=global_location_msg.pose.pose.position.y,
            z=0.0  
        )
        t.transform.rotation = global_location_msg.pose.pose.orientation

        self.tf_br_w2bl.sendTransform(t)

        q = global_location_msg.pose.pose.orientation
        global_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f"Global location: \
                               x={global_location_msg.pose.pose.position.x}, \
                                y={global_location_msg.pose.pose.position.y}, \
                                    yaw={global_yaw:2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AutocarTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
