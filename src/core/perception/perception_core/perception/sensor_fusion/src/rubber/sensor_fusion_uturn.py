#!/usr/bin/env python3

import numpy as np
import time

from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

#from motrackers import CentroidTracker, CentroidKF_Tracker, SORT, IOUTracker
#from motrackers.utils import draw_tracks

from perception.sensor_fusion.src.rubber.sensor_fusion_handler import *

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
import message_filters
from std_msgs.msg import Bool

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion_uturn')
        
        # Lane Mission Controller에서 오는 enable 상태
        self.is_enabled = False
        
        # Enable 신호 구독
        self.enable_sub = self.create_subscription(
            Bool,
            '/mission/rubber_uturn/enable',
            self.callback_enable,
            10)

        self.bridge = CvBridge()

        # 0830
        #self.intrinsic_right = np.array([[557.806489985562,   0., 313.278798427776, 0.],
        #                                  [0., 558.033519869795, 221.832568485757, 0.],
        #                                 [0., 0., 1., 0.]])
        #self.extrinsic_right = self.rtlc(alpha=np.radians(-87.2),
        #                                beta=np.radians(-49.7),
        #                               gamma=np.radians(179.1),
        #                               tx=-0.707366566360487, ty=0.449338070735928, tz=-0.260777068981733)
        #self.extrinsic_right = np.array([
        #   [-0.646827193433469, -0.762331655475608,  0.0215645286245695, -0.707366566360487],
        #   [ 0.0105802657440605, -0.0372435842785907, -0.999250205607619,  0.449338070735928],
        #    [ 0.762563203814455,  -0.646114047587500,  0.0321558345923653, -0.260777068981733],
        #    [ 0.0,                 0.0,                 0.0,                 1.0]
        #])
        # 새로운 파라미터 (역행렬로 사용할 것)
        #self.extrinsic_right = np.array([
        #    [-0.61872138,  0.00947078,  0.78555341, -0.73144129],
        #    [-0.78552194, -0.02247129, -0.61842568, -0.53301979],
        #    [ 0.01179542, -0.99970263,  0.02134297,  0.48576085],
        #    [ 0.0,                 0.0,                 0.0,                 1.0]
        #])

        # 0904
        # self.intrinsic_right = np.array([[619.6081,   0., 314.5476, 0.],
        #                                  [0., 619.7349, 224.2868, 0.],
        #                                  [0., 0., 1., 0.]])
        # self.extrinsic_right = np.array([
        #     [-0.618721384543836,  -0.785521938769354,  0.011795423697079, -0.876986864685114],
        #     [0.009470779095116, -0.022471287889330, -0.999702628567079, 0.480566072341194],
        #     [0.785553405346505, -0.618425682626967,  0.021342971111821, 0.234585492330497],
        #     [ 0.0,                 0.0,                 0.0,                 1.0]
        # ])

        # 1002
        self.intrinsic_right = np.array([[587.948744095411,   0., 314.5476, 0.],
                                         [0., 588.336739734481, 224.2868, 0.],
                                         [0., 0., 1., 0.]])
        self.extrinsic_right = np.array([[-0.638771164943078,	-0.769271226942017,	0.0138988573770317,	-0.891555405478185],
                                        [-0.00559926493774663,	-0.0134162558083088,	-0.999894320572051,	0.438251648438707],
                                        [0.769376401424830,	-0.638781483356562,	0.00426256394321395,	0.0788470712903576],
                                        [0,	0,	0,	1]])

        # ROS
        # Subscriber
        self.cluster_sub = message_filters.Subscriber(self, MarkerArray, '/adaptive_clustering/markers')
        self.bbox_sub = message_filters.Subscriber(self, PoseArray, '/bounding_boxes/tracked')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cluster_sub, self.bbox_sub], queue_size=10,
            slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_fusion)

        # Publisher
        self.fusion_pub = self.create_publisher(MarkerArray, '/sensor_fusion/rubber_cones', 10)
        self.clusters_2d_pub = self.create_publisher(PoseArray, '/clusters_2d', 10)

        self.get_logger().info('🚀  Camera & 3D LiDAR fusion node started.')

    def callback_enable(self, msg):
        """Lane Mission Controller에서 오는 enable 신호 콜백"""
        was_enabled = self.is_enabled
        self.is_enabled = msg.data
        
        if was_enabled != self.is_enabled:
            status = "활성화" if self.is_enabled else "비활성화"
            self.get_logger().info(f'라바콘(유턴) 센서퓨전 {status}')

    def callback_fusion(self, cluster_msg, bbox_msg):
        # Enable 상태 확인 - 비활성화 상태에서는 처리 건너뛰기
        if not self.is_enabled:
            return
            
        first_time = time.perf_counter()

        # Clustering points to np array
        clusters = cluster_for_fusion(cluster_msg) # 클러스터링 중점을 계산 (3D)

        # 2D bounding boxes
        #left_bboxes, left_labels, right_bboxes, right_labels = bounding_boxes(bbox_msg)
        right_bboxes, right_labels = bounding_boxes(bbox_msg)
        
        # 3D BBOX to Pixel Frame
        #clusters_2d_left, valid_left = projection_3d_to_2d(clusters, self.intrinsic_left, self.extrinsic_left)
        clusters_2d_right, valid_right = projection_3d_to_2d(clusters, self.intrinsic_right, self.extrinsic_right)
        
        # 디버깅
        self.get_logger().info(f"[SF] valid_right: {np.count_nonzero(valid_right)}/{len(valid_right)}")
        try:
            P = np.c_[clusters.T[:,:3], np.ones((clusters.shape[1], 1))]
            Xc = (self.extrinsic_right @ P.T).T
            Zc = Xc[:, 2]
            self.get_logger().info(f"[SF] Zc>0: {np.count_nonzero(Zc>0)}/{len(Zc)} "
                                   f"(minZ={Zc.min():.3f}, maxZ={Zc.max():.3f})")
        except Exception as e:
            self.get_logger().warn(f"[SF] cam-space debug skipped: {e}")
        if clusters_2d_right.size > 0:
            xs, ys = clusters_2d_right[:,0], clusters_2d_right[:,1]
            self.get_logger().info(f"[SF] clusters_2d_right x:[{np.nanmin(xs):.1f},{np.nanmax(xs):.1f}] "
                                   f"y:[{np.nanmin(ys):.1f},{np.nanmax(ys):.1f}]")
            
        
        # Sensor Fusion (Hungarian Algorithm)
        #matched_left = hungarian_match(clusters_2d_left, left_bboxes, left_labels, distance_threshold=120)
        matched_right = hungarian_match(clusters_2d_right, right_bboxes, right_labels, distance_threshold=80)


        # 디버깅
        bbs = np.asarray(right_bboxes, dtype=float)
        if bbs.ndim == 2 and bbs.shape[1] == 4:
            bbs = np.c_[(bbs[:,0]+bbs[:,2])/2.0, (bbs[:,1]+bbs[:,3])/2.0]
        pts = np.asarray(clusters_2d_right, dtype=float)
        if bbs.ndim == 2 and bbs.shape[1] == 4:
            bbs = np.c_[(bbs[:,0]+bbs[:,2])/2.0, (bbs[:,1]+bbs[:,3])/2.0]  # (M,2)
        if pts.size > 0 and bbs.size > 0:
            cost = distance_matrix(pts, bbs)              # 중심점 L2 거리
            rows, cols = linear_sum_assignment(cost)      # 동일한 규칙으로 할당만 재현
            lines = []
            for i, j in zip(rows, cols):
                lines.append(
                    f"({i}->{j}) d={cost[i,j]:.1f} "
                    f"pt=({pts[i,0]:.1f},{pts[i,1]:.1f}) "
                    f"ctr=({bbs[j,0]:.1f},{bbs[j,1]:.1f})"
                )
            self.get_logger().info("[SF] assignments: " + " | ".join(lines))
        self.get_logger().info(f"[SF] #right_bboxes={len(right_bboxes)}, labels={set(right_labels)}")
        self.get_logger().info(f"[SF] matched_right len={len(matched_right)}")
        self.get_logger().info(f"shape={right_bboxes.shape}")


        #labels_left = get_label(matched_left, valid_left)
        labels_right = get_label(matched_right, valid_right)
        #labels = [i if i != -1 else j if j != -1 else -1
        #          for i, j in zip(labels_left, labels_right)]
        labels = labels_right

        self.get_logger().debug(f'라벨 개수: {len(labels)}, {len(clusters.T[:,:3])}')

        # ROS Publish (Result of sensor fusion)
        fusion_markers = MarkerArray()
        
        # fusion_unmatched_markers = MarkerArray()
        blue_marker   = self.make_marker((0.0, 0.0, 1.0), "blue", 1)
        # yellow_marker = self.make_marker((1.0, 1.0, 0.0), "yellow", 2)
        # red_marker    = self.make_marker((1.0, 0.0, 0.0), "red", 3)
        white_marker  = self.make_marker((1.0, 1.0, 1.0), "white", 4)

        label_clusters(clusters.T[:,:3], labels, blue_marker, white_marker)

        # 디버깅
        self.get_logger().info(f"[SF] blue:{len(blue_marker.points)} "
                       f"white:{len(white_marker.points)}")

        fusion_markers.markers.extend([blue_marker, white_marker])
        self.fusion_pub.publish(fusion_markers)

        # ROS Publish (Projected clusters to 2D frame) 
        #clusters_2d_right[:, 0] += 640
        #clusters_2d = np.vstack([clusters_2d_left, clusters_2d_right])
        clusters_2d = clusters_2d_right

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
        Rx180 = np.array([[1, 0, 0, 0],
                         [0, np.cos(np.deg2rad(180)), -np.sin(np.deg2rad(180)), 0],
                         [0, np.sin(np.deg2rad(180)),  np.cos(np.deg2rad(180)), 0],
                         [0, 0, 0, 1]])
        T = np.array([[1, 0, 0, tx],
                      [0, 1, 0, ty],
                      [0, 0, 1, tz],
                      [0, 0, 0, 1]])
        #return Rzg @ Rxa @ Ryb @ Ry90 @ Rx90 @ T
        return Rzg @ Ryb @ Rxa @ T
        #return Rzg @ Rxa @ Ryb @ Rx180 @ T

    def make_marker(self, color, ns, mid):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.header.frame_id = 'velodyne'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        #  marker.id = int(sum(color) * 10000)
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
