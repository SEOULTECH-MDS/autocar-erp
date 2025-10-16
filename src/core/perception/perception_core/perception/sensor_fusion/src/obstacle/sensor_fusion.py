#!/usr/bin/env python3

import numpy as np
import time

from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

#from motrackers import CentroidTracker, CentroidKF_Tracker, SORT, IOUTracker
#from motrackers.utils import draw_tracks

from perception.sensor_fusion.src.obstacle.sensor_fusion_handler import *

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
import message_filters

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        self.bridge = CvBridge()

        # 0910
        # self.intrinsic = np.array([[558.376703987462, 0.0, 319.986731244040, 0.0],
        #                            [0.0, 557.567899661057, 238.850095504175, 0.0],
        #                            [0.0, 0.0, 1.0, 0.0]])
        # self.extrinsic = np.array([[-0.000669194348398178, -0.999833775455431, 0.0182201437278294, 0.0305848258731539],
        #                            [-0.0280510772828255, -0.0181942095036230, -0.999440897604161, 0.932753726748920],
        #                            [0.999606267108304, -0.00117991486004819, -0.0280342390431061, 0.248821155889658],
        #                            [0, 0, 0, 1]])

        # 1002
        self.intrinsic = np.array([[544.919521211672, 0.0, 306.352064813878, 0.0],
                                   [0.0, 544.019688159067, 245.365671510212, 0.0],
                                   [0.0, 0.0, 1.0, 0.0]])
        self.extrinsic = np.array([[0.0506487743641060, -0.998484984623336, 0.0215043515863724, 0.0920721610972854],
                                   [-0.0296475216862914, -0.0230256896413618, -0.999295172646401, 0.831791576567763],
                                   [0.998276377619582, 0.0499755249925013, -0.0307688281942799, 0.0776848798477488],
                                   [0, 0, 0, 1]])

        # ROS
        # Subscriber
        self.cluster_sub = message_filters.Subscriber(self, MarkerArray, '/adaptive_clustering/markers')
        self.bbox_sub_drum = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/drum')
        self.bbox_sub_car = message_filters.Subscriber(self, PoseArray, "/bounding_boxes/car")

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cluster_sub, self.bbox_sub_drum, self.bbox_sub_car], queue_size=10,
            slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_fusion)

        # Publisher
        self.fusion_pub = self.create_publisher(MarkerArray, '/sensor_fusion/obstacles', 10)
        self.clusters_2d_pub = self.create_publisher(PoseArray, '/clusters_2d', 10)

        self.get_logger().info('🚀  Camera & 3D LiDAR fusion node started.')

    def callback_fusion(self, cluster_msg, bbox_msg_drum, bbox_msg_car):        
        first_time = time.perf_counter()

        # Clustering points to np array
        clusters = cluster_for_fusion(cluster_msg) # 클러스터링 중점을 계산 (3D)

        # 2D bounding boxes
        drum_bboxes, drum_labels = bounding_boxes(bbox_msg_drum)
        car_bboxes, car_labels = bounding_boxes(bbox_msg_car)

        # 두 bbox 데이터를 합치기
        if drum_bboxes.size > 0 and car_bboxes.size > 0:
            all_bboxes = np.vstack([drum_bboxes, car_bboxes])
            all_labels = drum_labels + car_labels
        elif drum_bboxes.size > 0:
            all_bboxes = drum_bboxes
            all_labels = drum_labels
        elif car_bboxes.size > 0:
            all_bboxes = car_bboxes
            all_labels = car_labels
        else:
            all_bboxes = np.empty((0, 2))
            all_labels = []

        # 3D BBOX to Pixel Frame
        clusters_2d, valid_indicies = projection_3d_to_2d(clusters, self.intrinsic, self.extrinsic)
        
        # # 디버깅
        # self.get_logger().info(f"[SF] valid_right: {np.count_nonzero(valid_indicies)}/{len(valid_indicies)}")
        # try:
        #     P = np.c_[clusters.T[:,:3], np.ones((clusters.shape[1], 1))]
        #     Xc = (self.extrinsic @ P.T).T
        #     Zc = Xc[:, 2]
        #     self.get_logger().info(f"[SF] Zc>0: {np.count_nonzero(Zc>0)}/{len(Zc)} "
        #                            f"(minZ={Zc.min():.3f}, maxZ={Zc.max():.3f})")
        # except Exception as e:
        #     self.get_logger().warn(f"[SF] cam-space debug skipped: {e}")
        # if clusters_2d.size > 0:
        #     xs, ys = clusters_2d[:,0], clusters_2d[:,1]
        #     self.get_logger().info(f"[SF] clusters_2d x:[{np.nanmin(xs):.1f},{np.nanmax(xs):.1f}] "
        #                            f"y:[{np.nanmin(ys):.1f},{np.nanmax(ys):.1f}]")
             
        # Sensor Fusion (Hungarian Algorithm)
        #matched = hungarian_match(clusters_2d, rubber_bboxes, rubber_labels, distance_threshold=30)
        matched = hungarian_match(clusters_2d, all_bboxes, all_labels, distance_threshold=30)

        # # 디버깅
        # bbs = np.asarray(all_bboxes, dtype=float)
        # if bbs.ndim == 2 and bbs.shape[1] == 4:
        #     bbs = np.c_[(bbs[:,0]+bbs[:,2])/2.0, (bbs[:,1]+bbs[:,3])/2.0]
        # pts = np.asarray(clusters_2d, dtype=float)
        # if bbs.ndim == 2 and bbs.shape[1] == 4:
        #     bbs = np.c_[(bbs[:,0]+bbs[:,2])/2.0, (bbs[:,1]+bbs[:,3])/2.0]  # (M,2)
        # if pts.size > 0 and bbs.size > 0:
        #     cost = distance_matrix(pts, bbs)              # 중심점 L2 거리
        #     rows, cols = linear_sum_assignment(cost)      # 동일한 규칙으로 할당만 재현
        #     lines = []
        #     for i, j in zip(rows, cols):
        #         lines.append(
        #             f"({i}->{j}) d={cost[i,j]:.1f} "
        #             f"pt=({pts[i,0]:.1f},{pts[i,1]:.1f}) "
        #             f"ctr=({bbs[j,0]:.1f},{bbs[j,1]:.1f})"
        #         )
        #     self.get_logger().info("[SF] assignments: " + " | ".join(lines))
        # self.get_logger().info(f"[SF] #right_bboxes={len(all_bboxes)}, labels={set(all_labels)}")
        # self.get_logger().info(f"[SF] matched_right len={len(matched)}")
        # self.get_logger().info(f"shape={all_bboxes.shape}")


        labels = get_label(matched, valid_indicies)

        self.get_logger().debug(f'라벨 개수: {len(labels)}, {len(clusters.T[:,:3])}')

        # ROS Publish (Result of sensor fusion)
        fusion_markers = MarkerArray()
        
        # fusion_unmatched_markers = MarkerArray()
        blue_marker   = self.make_marker((0.0, 0.0, 1.0), "blue", 1)
        yellow_marker = self.make_marker((1.0, 1.0, 0.0), "yellow", 2)
        white_marker  = self.make_marker((1.0, 1.0, 1.0), "white", 3)

        label_clusters(clusters.T[:,:3], labels, blue_marker, yellow_marker, white_marker)

        # 차량 감지 시 2m 앞에 노란색 마커 추가
        if len(labels) > 0:
            cluster_points = clusters.T[:,:3]  # 3D 클러스터 포인트들
            for i, label in enumerate(labels):
                if label == 2:  # 차량 라벨
                    # 현재 차량 위치에서 x축 방향으로 2m 앞 계산
                    car_position = cluster_points[i]
                    virtual_position = car_position.copy()
                    virtual_position[0] += 2.0  # x축 방향으로 2m 앞

                    # 기존 노란색 마커에 가상 포인트 추가
                    from geometry_msgs.msg import Point
                    virtual_point = Point()
                    virtual_point.x = float(virtual_position[0])
                    virtual_point.y = float(virtual_position[1])
                    virtual_point.z = float(virtual_position[2])
                    yellow_marker.points.append(virtual_point)
                    
                    self.get_logger().info(f"[SF] 차량 감지: ({car_position[0]:.2f}, {car_position[1]:.2f}, {car_position[2]:.2f}) "
                                         f"-> 가상 위치: ({virtual_position[0]:.2f}, {virtual_position[1]:.2f}, {virtual_position[2]:.2f})")

        # 디버깅
        # self.get_logger().info(f"[SF] blue:{len(blue_marker.points)} "
        #                f"yellow:{len(yellow_marker.points)} white:{len(white_marker.points)}")

        fusion_markers.markers.extend([blue_marker, yellow_marker, white_marker])
        self.fusion_pub.publish(fusion_markers)

        # ROS Publish (Projected clusters to 2D frame) 
        clusters_2d = clusters_2d

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

    def make_marker(self, color, ns, mid):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.header.frame_id = 'velodyne'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        # marker.id = int(sum(color) * 10000)
        marker.ns = ns
        marker.id = mid * 10000
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