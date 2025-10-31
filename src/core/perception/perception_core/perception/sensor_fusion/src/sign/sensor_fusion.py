#!/usr/bin/env python3

import numpy as np
import time

# from motrackers import CentroidTracker, CentroidKF_Tracker, SORT, IOUTracker
# from motrackers.utils import draw_tracks

from perception.sensor_fusion.src.sign.sensor_fusion_handler import *

# ROS
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String, Int32, Bool

 
# 배달미션 파라미터 CARLA
# self.intrinsic = np.array([[515.19, 0.0, 333.26, 0.0],
#                                 [0.0, 632.87, 271.89, 0.0],
#                                 [0.0, 0.0, 1.0, 0.0]])

# self.extrinsic = self.rtlc(alpha = np.radians(0.2),
#                                         beta = np.radians(319.1),
#                                         gamma = np.radians(0.9), 
#                                         tx = 0.673, ty = -0.753, tz = -0.047)

# 배달미션 파라미터 2 (라이다 앞으로 옮김) CARLA
# self.intrinsic = np.array([[683.45, 0.0, 327.82, 0.0],
#                                [0.0, 576.92, 247.73, 0.0],
#                                [0.0, 0.0, 1.0, 0.0]])

# self.extrinsic = self.rtlc(alpha = np.radians(0.0),
#                                         beta = np.radians(336.6),
#                                         gamma = np.radians(0), 
#                                         tx = 2.46, ty = 0.826, tz = -0.083)

# 배달미션 파라미터 

# self.intrinsic = np.array([[[560.75, 0.0, 305.76, 0.0],
                            # [0.0, 538.46, 293.67, 0.0],
                            # [0.0, 0.0, 1.0, 0.0]])

# self.extrinsic = self.rtlc(alpha = np.radians(20.5),
#                                         beta = np.radians(332.7),
#                                         gamma = np.radians(348.9), 
#                                         tx = 1.52, ty = 0.496, tz = -0.808)


class SensorFusion(Node):
    def __init__(self):
        self.bridge = CvBridge()
        
        # 2024
        #self.intrinsic = np.array([[378.68261719, 0.0, 328.19930137, 0.0],
        #                                    [0.0, 443.68624878, 153.57524293, 0.0],
        #                                    [0.0, 0.0, 1.0, 0.0]])
        #
        #self.extrinsic = self.rtlc(alpha = np.radians(2.2),
        #                                        beta = np.radians(326.4),
        #                                        gamma = np.radians(359.4), 
        #                                        tx = 0.965, ty = 0.22, tz = -0.7)

        # 0904
        # self.intrinsic = np.array([[619.6081,   0., 314.5476, 0.],
        #                                  [0., 619.7349, 224.2868, 0.],
        #                                  [0., 0., 1., 0.]])
        # self.extrinsic = np.array([
        #     [-0.618721384543836,  -0.785521938769354,  0.011795423697079, -0.876986864685114],
        #     [0.009470779095116, -0.022471287889330, -0.999702628567079, 0.480566072341194],
        #     [0.785553405346505, -0.618425682626967,  0.021342971111821, 0.234585492330497],
        #     [ 0.0,                 0.0,                 0.0,                 1.0]])

        self.intrinsic = np.array([[558.376703987462, 0.0, 319.986731244040, 0.0],
                                   [0.0, 557.567899661057, 238.850095504175, 0.0],
                                   [0.0, 0.0, 1.0, 0.0]])
        self.extrinsic = np.array([[-0.000669194348398178, -0.999833775455431, 0.0182201437278294, 0.0305848258731539],
                                   [-0.0280510772828255, -0.0181942095036230, -0.999440897604161, 0.932753726748920],
                                   [0.999606267108304, -0.00117991486004819, -0.0280342390431061, 0.248821155889658],
                                   [0, 0, 0, 1]])

        self.bboxes = None
        self.clusters_2d = None
        self.target_sign = None
    
        # ROS
        super().__init__('sensor_fusion_sign')
        
        # Lane Mission Controller에서 오는 enable 상태
        self.is_enabled = False
        
        # Enable 신호 구독
        self.enable_sub = self.create_subscription(
            Bool,
            '/mission/sign/enable',
            self.callback_enable,
            10)

        self.cluster_sub = message_filters.Subscriber(self, MarkerArray, '/adaptive_clustering/markers')
        self.bbox_sub = message_filters.Subscriber(self, PoseArray, "/bounding_boxes/deliver")

        self.sync = message_filters.ApproximateTimeSynchronizer([self.cluster_sub, self.bbox_sub], queue_size=20, slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_fusion)
        
        self.image_sub = self.create_subscription(Image, '/yolo/sign', self.callback_img, 10)
        self.target_sign_sub = self.create_subscription(Int32, '/target_sign', self.callback_targetsign, 10)

        self.result_img_pub = self.create_publisher(Image, '/result_img', 10)
        self.deliverysign_spot_pub = self.create_publisher(PoseArray, '/deliverysign_spot', 10)

        self.get_logger().info('\033[1;33mStarting camera + LiDAR sensor-fusion (ROS 2)…\033[0m')
        return
    
    def callback_enable(self, msg):
        """Lane Mission Controller에서 오는 enable 신호 콜백"""
        was_enabled = self.is_enabled
        self.is_enabled = msg.data
        
        if was_enabled != self.is_enabled:
            status = "활성화" if self.is_enabled else "비활성화"
            self.get_logger().info(f'표지판 센서퓨전 {status}')
    
    def callback_img(self, img_msg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        if self.clusters_2d is not None and self.bboxes is not None:
            visualize_cluster_2d(self.clusters_2d, img)
            visualize_bbox(self.bboxes, img)

        result_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        result_img.header.stamp = self.get_clock().now().to_msg()
        self.result_img_pub.publish(result_img)

    def callback_targetsign(self, sign_msg):
        self.target_sign = sign_msg.data
        
    def callback_fusion(self, cluster_msg, bbox_msg):
        # Enable 상태 확인 - 비활성화 상태에서는 처리 건너뛰기
        if not self.is_enabled:
            return
            
        first_time = time.perf_counter()

        # Clustering points to np array
        clusters = cluster_for_fusion(cluster_msg) # 클러스터링 중점을 계산 (3D)
        # self.get_logger().info(f'Clusters: {clusters.T[:,:3]}')
        
        # 2D bounding boxes
        bboxes, bboxes_label = bounding_boxes(bbox_msg)
        self.bboxes = bboxes
        # 3D BBOX to Pixel Frame
        clusters_2d, valid_indicies = projection_3d_to_2d(clusters, self.intrinsic, self.extrinsic)
        self.clusters_2d = clusters_2d
        # self.get_logger().info(f"Clusters 2D: {clusters_2d}, Valid indices: {valid_indicies}")

        # Sensor Fusion (Hungarian Algorithm)
        matched = hungarian_match(clusters_2d, bboxes, bboxes_label, distance_threshold=30)
        self.get_logger().info(f'Matched indices: {matched}')
        
        labels = get_label(matched, valid_indicies)
        self.get_logger().info(f'Labels: {labels}')

        target_clusters = []
        delivery_pose_array = PoseArray()
        delivery_pose_array.header.frame_id = 'velodyne'
        delivery_pose_array.header.stamp = self.get_clock().now().to_msg()
        for idx, id in enumerate(labels):
            if id == 0:
                target_clusters.append(clusters.T[:,:3][idx])
            elif id == 1:
                target_clusters.append(clusters.T[:,:3][idx])
            elif id == 2:
                target_clusters.append(clusters.T[:,:3][idx])
            elif id == self.target_sign:
                target_clusters.append(clusters.T[:,:3][idx])    
            
        self.get_logger().info(f'Target clusters: {target_clusters}')


        if target_clusters:
            # 각 클러스터 좌표에 대해 Pose를 생성하여 PoseArray에 추가
            for cluster in target_clusters:
                pose = Pose()
                pose.position.x = cluster[0]  # x 좌표
                pose.position.y = cluster[1]  # y 좌표
                pose.position.z = cluster[2]  # z 좌표

                # PoseArray에 Pose 추가
                delivery_pose_array.poses.append(pose)
        
            self.deliverysign_spot_pub.publish(delivery_pose_array)
        
        self.get_logger().info(f'fusion 좌표: {target_clusters}')
        self.get_logger().info(f'소요시간: {time.perf_counter() - first_time:.5f}')
        return
    
    def rtlc(self, alpha, beta, gamma, tx, ty, tz):              
        Rxa = np.array([[1, 0, 0,0],
                        [0, np.cos(alpha), -np.sin(alpha),0],
                        [0, np.sin(alpha), np.cos(alpha),0],
                        [0,0,0,1]])

        Ryb = np.array([[np.cos(beta), 0, np.sin(beta),0],
                        [0, 1, 0,0],
                        [-np.sin(beta), 0, np.cos(beta),0],
                        [0,0,0,1]])

        Rzg = np.array([[np.cos(gamma), -np.sin(gamma), 0, 0],
                [np.sin(gamma), np.cos(gamma), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
        
        Ry90 = np.array([[np.cos(np.deg2rad(-90)), 0, np.sin(np.deg2rad(-90)),0],
                         [0, 1, 0,0],
                         [-np.sin(np.deg2rad(-90)), 0, np.cos(np.deg2rad(-90)),0],
                         [0,0,0,1]])
                 
        Rx90= np.array([[1, 0, 0,0],
                        [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)),0],
                        [0, np.sin(np.deg2rad(90)), np.cos(np.deg2rad(90)),0],
                        [0,0,0,1]])
        
        T = np.array([[1, 0, 0, tx],      
                      [0, 1, 0, ty],                  
                      [0, 0, 1, tz],                       
                      [0, 0, 0, 1]]) 
        
        rtlc = Rzg@Rxa@Ryb@Ry90@Rx90@T
        return rtlc
    
    def make_marker(self, color):
        marker = Marker()
        marker.action = marker.ADD
        marker.type = marker.POINTS
        marker.header.frame_id = "velodyne"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.lifetime = Duration(seconds=0.1).to_msg()
        marker.id = int((color[0] + color[1] + color[2]) * 10000)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.orientation.w = 1.0
        return marker
    
    @staticmethod
    def make_pose_array(points):
        pose_array = PoseArray()
        pose_array.header.stamp = Clock(clock_type=ClockType.ROS_TIME).now().to_msg()
        pose_array.header.frame_id = 'yolo'
        for x, y in points:
            pose = Pose()
            pose.orientation.x = x
            pose.orientation.y = y
            
            pose_array.poses.append(pose)
        return pose_array
    
def main():
    rclpy.init()
    node = SensorFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()