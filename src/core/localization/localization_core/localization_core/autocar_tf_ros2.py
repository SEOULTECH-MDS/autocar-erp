#! /usr/bin/env python3

import pyproj
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AutocarTFPublisher(Node):
    def __init__(self):
        super().__init__('autocar_tf_publisher')
        
        # QoS Profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # TF Broadcasters
        self.static_br = StaticTransformBroadcaster(self)
        self.dynamic_br = TransformBroadcaster(self)
        
        # Parameters
        self.declare_parameter('map_origin_lat', 37.630117)
        self.declare_parameter('map_origin_lon', 127.081431)  
        
        # Subscriber
        self.location_sub = self.create_subscription(
            Odometry, 
            '/autocar/location', 
            self.location_callback, 
            qos
        )
        
        # Setup map frame
        self.setup_map_frame()
        self.get_logger().info('TF Publisher Node has been started.')
        
    def setup_map_frame(self):
        lat = self.get_parameter('map_origin_lat').value
        lon = self.get_parameter('map_origin_lon').value
        
        # Lat/Lon to UTM conversion for map origin
        proj_string = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        utm_proj = pyproj.Proj(proj_string)
        self.map_origin_x, self.map_origin_y = utm_proj(lon, lat)
        
        # Publish static transform from 'world' to 'map'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.translation = Vector3(x=float(self.map_origin_x), y=float(self.map_origin_y), z=0.0)
        t.transform.rotation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self.static_br.sendTransform(t)
        self.get_logger().info(f"Published static TF: world -> map at {self.map_origin_x}, {self.map_origin_y}")

    def location_callback(self, msg):
        # Publish dynamic transform from 'map' to 'base_link'
        t = TransformStamped()
        # Use odometry timestamp for consistency with pose
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        vehicle_utm_x = msg.pose.pose.position.x
        vehicle_utm_y = msg.pose.pose.position.y
        
        vehicle_map_x = vehicle_utm_x - self.map_origin_x
        vehicle_map_y = vehicle_utm_y - self.map_origin_y
        
        t.transform.translation = Vector3(x=vehicle_map_x, y=vehicle_map_y, z=0.28)
        t.transform.rotation = msg.pose.pose.orientation
        
        self.dynamic_br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AutocarTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
