#!/usr/bin/env python3

import numpy as np
import time

#from motrackers import CentroidTracker, CentroidKF_Tracker, SORT, IOUTracker
#from motrackers.utils import draw_tracks

from perception.sensor_fusion.src.object.sensor_fusion_handler import *

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
import message_filters

# 2024 파라미터
#self.intrinsic_left = np.array([[381.18310547,   0.,         319.02992935, 0.],
#                    [0. ,        448.0055542, 207.31755552, 0.],
#                    [0.000000, 0.000000, 1.000000, 0.]])
#
#self.extrinsic_left = self.rtlc(alpha = np.radians(38.0),
#                                beta = np.radians(26.0),
#                                gamma = np.radians(4.9), 
#                                tx = 0.965, ty = -0.23, tz = -0.95)
#
#self.intrinsic_right = np.array([[378.68261719,   0.,         328.19930137, 0.],
#                    [0. ,        443.68624878, 153.57524293, 0.],
#                    [0.000000, 0.000000, 1.000000, 0.]])
#
#self.extrinsic_right = self.rtlc(alpha = np.radians(36.8),
#                                    beta = np.radians(-29.4),
#                                    gamma = np.radians(-5.5), 
#                                    tx = 0.965, ty = 0.218, tz = -0.965)

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        self.bridge = CvBridge()

        self.intrinsic_left = np.array([[381.18310547,   0., 319.02992935, 0.],
                                         [0., 448.0055542, 207.31755552, 0.],
                                         [0., 0., 1., 0.]])
        self.extrinsic_left = self.rtlc(alpha=np.radians(38.0),
                                        beta=np.radians(26.0),
                                        gamma=np.radians(4.9),
                                        tx=0.965, ty=-0.23, tz=-0.95)

        self.intrinsic_right = np.array([[378.68261719,   0., 328.19930137, 0.],
                                          [0., 443.68624878, 153.57524293, 0.],
                                          [0., 0., 1., 0.]])
        self.extrinsic_right = self.rtlc(alpha=np.radians(36.8),
                                         beta=np.radians(-29.4),
                                         gamma=np.radians(-5.5),
                                         tx=0.965, ty=0.218, tz=-0.965)

        # ROS
        # Subscriber
        # self.cluster_sub = message_filters.Subscriber(self, MarkerArray, '/adaptive_clustering/markers')
        self.bbox_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/tracked')
        self.cluster_sub = message_filters.Subscriber(self, MarkerArray, '/markers')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cluster_sub, self.bbox_sub], queue_size=10,
            slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_fusion)

        # Publisher
        self.fusion_pub = self.create_publisher(MarkerArray, '/sensor_fusion/rubber_cones', 10)
        self.clusters_2d_pub = self.create_publisher(PoseArray, '/clusters_2d', 10)

        self.get_logger().info('🚀  Camera & 3D LiDAR fusion node started.')

    def callback_fusion(self, cluster_msg, bbox_msg):
        first_time = time.perf_counter()

        # Clustering points to np array
        clusters = cluster_for_fusion(cluster_msg) # 클러스터링 중점을 계산 (3D)

        # 2D bounding boxes
        left_bboxes, left_labels, right_bboxes, right_labels = bounding_boxes(bbox_msg)

        # 3D BBOX to Pixel Frame
        clusters_2d_left, valid_left = projection_3d_to_2d(clusters, self.intrinsic_left, self.extrinsic_left)
        clusters_2d_right, valid_right = projection_3d_to_2d(clusters, self.intrinsic_right, self.extrinsic_right)

        # Sensor Fusion (Hungarian Algorithm)
        matched_left = hungarian_match(clusters_2d_left, left_bboxes, left_labels, distance_threshold=120)
        matched_right = hungarian_match(clusters_2d_right, right_bboxes, right_labels, distance_threshold=120)

        labels_left = get_label(matched_left, valid_left)
        labels_right = get_label(matched_right, valid_right)
        labels = [i if i != -1 else j if j != -1 else -1
                  for i, j in zip(labels_left, labels_right)]

        self.get_logger().debug(f'라벨 개수: {len(labels)}, {len(clusters.T[:,:3])}')

        # ROS Publish (Result of sensor fusion)
        fusion_markers = MarkerArray()
        # fusion_unmatched_markers = MarkerArray()
        blue_marker   = self.make_marker((0.0, 0.0, 1.0))
        yellow_marker = self.make_marker((1.0, 1.0, 0.0))
        white_marker  = self.make_marker((1.0, 1.0, 1.0))

        label_clusters(clusters.T[:,:3], labels, blue_marker, yellow_marker, white_marker)

        fusion_markers.markers.extend([blue_marker, yellow_marker, white_marker])
        self.fusion_pub.publish(fusion_markers)

        # ROS Publish (Projected clusters to 2D frame) 
        clusters_2d_right[:, 0] += 640
        clusters_2d = np.vstack([clusters_2d_left, clusters_2d_right])

        clusters_2d_msg = self.make_pose_array(clusters_2d)
        self.clusters_2d_pub.publish(clusters_2d_msg)

        self.get_logger().debug(f'소요 시간: {time.perf_counter() - first_time:.5f}s')

    # ───────────────── 유틸 ─────────────────
    def rtlc(self, alpha, beta, gamma, tx, ty, tz):
        Rxa = np.array([[1, 0, 0, 0],
                        [0, np.cos(alpha), -np.sin(alpha), 0],
                        [0, np.sin(alpha),  np.cos(alpha), 0],
                        [0, 0, 0, 1]])
        Ryb = np.array([[np.cos(beta), 0, np.sin(beta), 0],
                        [0, 1, 0, 0],
                        [-np.sin(beta), 0, np.cos(beta), 0],
                        [0, 0, 0, 1]])
        Rzg = np.array([[np.cos(gamma), -np.sin(gamma), 0, 0],
                        [np.sin(gamma),  np.cos(gamma), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        Ry90 = np.array([[np.cos(np.deg2rad(-90)), 0, np.sin(np.deg2rad(-90)), 0],
                         [0, 1, 0, 0],
                         [-np.sin(np.deg2rad(-90)), 0, np.cos(np.deg2rad(-90)), 0],
                         [0, 0, 0, 1]])
        Rx90 = np.array([[1, 0, 0, 0],
                         [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)), 0],
                         [0, np.sin(np.deg2rad(90)),  np.cos(np.deg2rad(90)), 0],
                         [0, 0, 0, 1]])
        T = np.array([[1, 0, 0, tx],
                      [0, 1, 0, ty],
                      [0, 0, 1, tz],
                      [0, 0, 0, 1]])
        return Rzg @ Rxa @ Ryb @ Ry90 @ Rx90 @ T

    def make_marker(self, color):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.header.frame_id = 'velodyne'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        marker.id = int(sum(color) * 10000)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.pose.orientation.w = 1.0
        return marker

    def make_pose_array(self, points):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'yolo'
        for x, y in points:
            pose = Pose()
            pose.orientation.x = float(x)
            pose.orientation.y = float(y)
            pose_array.poses.append(pose)
        return pose_array

def main():
    rclpy.init()
    sensor_fusion = SensorFusion()
    try:
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
