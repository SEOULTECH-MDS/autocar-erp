#!/usr/bin/env python3

import numpy as np
import time

from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

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
from std_msgs.msg import Int32MultiArray, String, Int32

class SensorFusion(Node):
    def __init__(self):
        self.bridge = CvBridge()

        #self.intrinsic = np.array([[378.68261719, 0.0, 328.19930137, 0.0],
        #                                    [0.0, 443.68624878, 153.57524293, 0.0],
        #                                    [0.0, 0.0, 1.0, 0.0]])
        #
        # self.extrinsic = self.rtlc(alpha = np.radians(2.2),
        #                                         beta = np.radians(326.4),
        #                                         gamma = np.radians(359.4), 
        #                                         tx = 0.965, ty = 0.22, tz = -0.7)
        
        self.intrinsic = np.array([[558.376703987462, 0.0, 319.986731244040, 0.0],
                                   [0.0, 557.567899661057, 238.850095504175, 0.0],
                                   [0.0, 0.0, 1.0, 0.0]])
        self.extrinsic = np.array([[-0.000669194348398178, -0.999833775455431, 0.0182201437278294, 0.0305848258731539],
                                   [-0.0280510772828255, -0.0181942095036230, -0.999440897604161, 0.932753726748920],
                                   [0.999606267108304, -0.00117991486004819, -0.0280342390431061, 0.248821155889658],
                                   [0, 0, 0, 1]])
        

        self.bboxes = None
        self.clusters_2d = None

        # ROS
        super().__init__('sensor_fusion')

        self.cluster_sub = message_filters.Subscriber(self, MarkerArray, '/adaptive_clustering/markers')
        self.bbox_sub_car = message_filters.Subscriber(self, PoseArray, "/bounding_boxes/car")
        self.bbox_sub_rubber = message_filters.Subscriber(self, PoseArray, "/bounding_boxes/rubber")

        self.sync = message_filters.ApproximateTimeSynchronizer([self.cluster_sub, self.bbox_sub_car, self.bbox_sub_rubber], queue_size=10, slop=0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_fusion)

        self.result_img_pub = self.create_publisher(Image, '/result_img', 10)
        self.fusion_pub = self.create_publisher(PoseArray, '/sensor_fusion/obstacle', 10)

        self.get_logger().info('\033[1;33mStarting camera + LiDAR sensor-fusion (ROS 2)…\033[0m')
        return
    
    def callback_img(self, img_msg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        if self.clusters_2d is not None and self.bboxes is not None:
            visualize_cluster_2d(self.clusters_2d, img)
            visualize_bbox(self.bboxes, img)

        result_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        result_img.header.stamp = self.get_clock().now().to_msg()
        self.result_img_pub.publish(result_img)
        
    def callback_fusion(self, cluster_msg, car_msg, rubber_msg):
        first_time = time.perf_counter()
        # self.get_logger().info(f"Received messages: {cluster_msg}, {rubber_msg}")

        # Clustering points to np array
        clusters = cluster_for_fusion(cluster_msg) # 클러스터링 중점을 계산 (3D)
        # self.get_logger().info(f'Clusters (3D): {clusters}')
        
        # 2D bounding boxes
        # car_bboxes, _ = bounding_boxes(car_msg)
        # rubber_bboxes, _ = bounding_boxes(rubber_msg)

        # car_bboxes = np.asarray(car_bboxes) if car_bboxes is not None else np.empty((0, 4))
        # rubber_bboxes = np.asarray(rubber_bboxes) if rubber_bboxes is not None else np.empty((0, 4))

        car_bboxes, car_labels = bounding_boxes(car_msg)
        rubber_bboxes, rubber_labels = bounding_boxes(rubber_msg)

        # 클래스 인덱스 강제 부여: 자동차=0, 라바콘=1
        # car_labels    = np.full((car_bboxes.shape[0],), 0, dtype=int)
        # rubber_labels = np.full((rubber_bboxes.shape[0],), 1, dtype=int)

        # 병합: bboxes, labels        
        if car_bboxes.size or rubber_bboxes.size:
            bboxes = np.vstack([car_bboxes, rubber_bboxes])
            bbox_labels = np.concatenate([car_labels, rubber_labels])
        else:
            bboxes = np.empty((0, 4))
            bbox_labels = np.empty((0,), dtype=int)

        self.bboxes = bboxes
        self.bbox_labels = bbox_labels

        # 3D BBOX to Pixel Frame
        clusters_2d, valid_indicies = projection_3d_to_2d(clusters, self.intrinsic, self.extrinsic)
        self.clusters_2d = clusters_2d
        self.get_logger().info(f'Clusters (2D): {clusters_2d}')

        # 바운딩 박스 중심좌표 계산 (2D)
        bbox_centers = []
        if bboxes.size > 0:
            if bboxes.shape[1] == 4:
                # (x1, y1, x2, y2) -> 중심
                bbox_centers = np.c_[(bboxes[:,0]+bboxes[:,2])/2.0, (bboxes[:,1]+bboxes[:,3])/2.0]
            elif bboxes.shape[1] == 2:
                bbox_centers = bboxes
        # 클러스터 투영 중심좌표 (2D)
        cluster_centers = clusters_2d if clusters_2d is not None else []

        # 실시간 로그 출력
        # self.get_logger().info(f"[2D] Cluster centers: {cluster_centers}")
        # self.get_logger().info(f"[2D] BBox centers: {bbox_centers}")
        # self.get_logger().info(f"[2D] BBox labels: {bbox_labels}")

        # Sensor Fusion (Hungarian Algorithm)
        matched = hungarian_match(clusters_2d, bboxes, bbox_labels, distance_threshold=120)
        #self.get_logger().info(f'Matched indices: {matched}')

        matched_labels = get_label(matched, valid_indicies)
        #self.get_logger().info(f'Labels: {matched_labels}')

        # 디버깅
        # bbs = np.asarray(bboxes, dtype=float)
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

        # valid_indicies True인 원본 clusters 인덱스 리스트
        # valid_indices_list = [i for i, v in enumerate(valid_indicies) if v]
        # self.get_logger().debug(f'valid_indices_list (len={len(valid_indices_list)}): {valid_indices_list}')

        obstacle_clusters = []
        obstacle_pose_array = PoseArray()
        obstacle_pose_array.header.frame_id = 'velodyne'
        obstacle_pose_array.header.stamp = self.get_clock().now().to_msg()

        # def get_xyz_from_clusters(cl, idx):
        #     # (N,3/4) 또는 (4,N) 대응
        #     if cl.ndim == 2:
        #         if cl.shape[1] in (3, 4):   # (N,3/4)
        #             return cl[idx, :3]
        #         if cl.shape[0] == 4:        # (4,N)
        #             return cl[:3, idx]
        #     return cl[idx, :3]

        # for ci, bi in matched:
        #     ci_val = ci[0] if isinstance(ci, (tuple, list, np.ndarray)) else ci
        #     if ci_val < 0 or ci_val >= len(valid_indices_list):
        #         self.get_logger().warning(f"ci {ci_val} out of valid_indices_list range -> skip")
        #         continue
        #     orig_idx = valid_indices_list[ci_val]
        #     try:
        #         xyz = get_xyz_from_clusters(clusters, orig_idx)
        #         x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        #     except Exception as e:
        #         self.get_logger().error(f"Failed to get xyz for orig_idx={orig_idx}: {e}")
        #         continue
        #     if not np.isfinite([x, y, z]).all():
        #         self.get_logger().warning(f"Non-finite xyz for orig_idx={orig_idx} -> skip")
        #         continue
        #     obstacle_clusters.append([x, y, z])

        #     pose = Pose()
        #     pose.position.x = x
        #     pose.position.y = y
        #     pose.position.z = z
        #     obstacle_pose_array.poses.append(pose)

        # for ci in matched:
        #     # 3D 중심 좌표
        #     x, y, z = get_xyz_from_clusters(clusters, ci).tolist()
        #     obstacle_clusters.append([x, y, z])
        #     # orig_idx = valid_indices_list[ci]
        #     # x, y, z = get_xyz_from_clusters(clusters, orig_idx).tolist()
        #     # obstacle_clusters.append([x, y, z])

        #     pose = Pose()
        #     pose.position.x = float(x)
        #     pose.position.y = float(y)
        #     pose.position.z = float(z)
        #     # 클래스 인덱스(0=car, 1=rubber)를 orientation.z에 저장 (w=1.0로 유지)
        #     #pose.orientation.z = float(labels[bi])
        #     pose.orientation.z = float(bbox_labels[ci])
        #     pose.orientation.w = 1.0
        #     obstacle_pose_array.poses.append(pose)

        # self.fusion_pub.publish(obstacle_pose_array)
        for idx, id in enumerate(bbox_labels):
            if id == 0:
                obstacle_clusters.append(clusters.T[:,:3][idx])
            elif id == 1:
                obstacle_clusters.append(clusters.T[:,:3][idx])
            elif id == 2:
                obstacle_clusters.append(clusters.T[:,:3][idx])
            # elif id == self.target_sign:
            #     obstacle_clusters.append(clusters.T[:,:3][idx])    

        # self.get_logger().debug(f'Obstacle clusters: {obstacle_clusters}')

        if obstacle_clusters:
            # 각 클러스터 좌표에 대해 Pose를 생성하여 PoseArray에 추가
            for cluster in obstacle_clusters:
                pose = Pose()
                pose.position.x = cluster[0]  # x 좌표
                pose.position.y = cluster[1]  # y 좌표
                pose.position.z = cluster[2]  # z 좌표

                # PoseArray에 Pose 추가
                obstacle_pose_array.poses.append(pose)
        
            self.fusion_pub.publish(obstacle_pose_array)

        # self.get_logger().info(f'fusion 좌표: {obstacle_clusters}')
        # self.get_logger().info(f'소요시간: {time.perf_counter() - first_time:.5f}')
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