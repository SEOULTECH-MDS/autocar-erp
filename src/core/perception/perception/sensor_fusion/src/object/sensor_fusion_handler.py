#!/usr/bin/env python3
import numpy as np
import cv2

from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment
from geometry_msgs.msg import Point

def cluster_for_fusion(cluster_msg):
    """
    Returns:
        Numpy array: ((x,y,z,1.0)...).T
    """
    
    clusters = np.empty((0,4), float) # 4 columns for x, y, z, 1.0
    for cluster in cluster_msg.markers:
        points = np.array([(point.x, point.y, point.z-0.10) for point in cluster.points])
        center = np.average(points, axis=0)
        center = np.append(center, 1.0)
            
        clusters = np.append(clusters, [center], axis=0)

    clusters = clusters.T
            
    return clusters

def bounding_boxes(bbox_msg):
    left_bboxes = np.empty((0,2))
    right_bboxes = np.empty((0,2))
    bboxes_label = []
    right_bboxes_label = []
    
    for bbox in bbox_msg.poses:
        if bbox.orientation.x < 640:
            # left_bboxes.append(((bbox.orientation.z + bbox.orientation.x)/2, (bbox.orientation.w + bbox.orientation.y)/2))
            left_bboxes = np.append(left_bboxes, [((bbox.orientation.z + bbox.orientation.x)/2, (bbox.orientation.w + bbox.orientation.y)/2)], axis=0)
            # left_bboxes.append([bbox.orientation.z, bbox.orientation.x, bbox.orientation.w, bbox.orientation.y])
            bboxes_label.append(int(bbox.position.x))
        else:
            right_bboxes = np.append(right_bboxes, [((bbox.orientation.z + bbox.orientation.x)/2 - 640, (bbox.orientation.w + bbox.orientation.y)/2)], axis=0)
            # right_bboxes.append([bbox.orientation.z, bbox.orientation.x, bbox.orientation.w, bbox.orientation.y])
            right_bboxes_label.append(int(bbox.position.x))

    return left_bboxes, bboxes_label, right_bboxes, right_bboxes_label

    
def projection_3d_to_2d(clusters, intrinsic, extrinsic):
    points_c = intrinsic @ (extrinsic @ clusters)
    center_x = points_c[0,:] / points_c[2,:] # center point in pixel frame
    center_y = points_c[1,:] / points_c[2,:]
    
    height, width = 360, 640
    valid_indicies = (center_x >= 0) & (center_x < width) & (center_y >=0) & (center_y < height)
    
    center_x = center_x[valid_indicies]
    center_y = center_y[valid_indicies]
    
    clusters_2d = np.vstack([center_x, center_y]).T

    return clusters_2d, valid_indicies

def hungarian_match(clusters_2d, bboxes, bbox_labels, distance_threshold = 80):
    cost = distance_matrix(clusters_2d, bboxes)
    # Hungarian algorithm
    assigned_clusters, assigned_bboxes = linear_sum_assignment(cost)
    
    # matched = []
    # unmatched_clusters = [c for c in range(clusters_2d.shape[0]) if c not in assigned_clusters]
    # unmatched_bboxes = [bb for bb in range(bboxes.shape[0]) if bb not in assigned_bboxes]

    # for c, bb in zip(assigned_clusters, assigned_bboxes):
    #     if cost[c,bb] > distance_threshold:
    #         unmatched_clusters.append(c)
    #         unmatched_bboxes.append(bb)
    #     else:
    #         matched.append((c, bb))

    matched = [-1] * len(clusters_2d)    
    for c, bb in zip(assigned_clusters, assigned_bboxes):
        if cost[c,bb] < distance_threshold:
            matched[c] =  bbox_labels[bb]
    return matched

def get_label(matched, valid_indicies):
    labels = []
    index = 0
    for i in range(len(valid_indicies)):
        if valid_indicies[i]:
            labels.append(matched[index])
            index += 1
        else:
            labels.append(-1)
    return labels

# def get_matched_clusters(matched, clusters_3d, bboxes_label):
#     clusters_labeled = []
#     labels = []
#     for i in range(matched.shape[0]):
#         c, bb = matched[i, :]
#         clusters_labeled.append([clusters_3d[c, 0], clusters_3d[c, 1], clusters_3d[c, 2]])
                
#         if bboxes_label[bb] == 0.0:
#             labels.append(0.0)
#         else:
#             labels.append(1.0)
#     return np.array(clusters_labeled), labels

# def get_unmatced_clusters(unmatched_clusters, clusters_3d):
#     unmatched_clusters_3d = []
#     for unmatched_cluster in unmatched_clusters:
#         unmatched_clusters_3d.append([clusters_3d[unmatched_cluster,0], clusters_3d[unmatched_cluster,1]])  
#     return np.array(unmatched_clusters_3d)

# def label_tracks(tracks):
#     blues = []
#     yellows = []
    
#     for _, _, x, y, _, _, _, lbl, _, _ in tracks:
#         point = Point()
#         point.x, point.y = x, y
        
#         if lbl == 0.0:
#             blues.append(point)
#         else:
#             yellows.append(point)
#     return blues, yellows

def label_clusters(clusters_3d, labels, blue_marker, yellow_marker, white_marker):
    for i in range(len(clusters_3d)):
        point = Point()
        point.x, point.y, point.z = clusters_3d[i, 0], clusters_3d[i, 1], clusters_3d[i, 2]
        
        if labels[i] == 0:
            blue_marker.points.append(point)
        elif labels[i] == 1:
            yellow_marker.points.append(point)
        else:
            white_marker.points.append(point)
    return

def visualize_cluster_2d(clusters_2d, img):
    for point in clusters_2d:
        cv2.circle(img, (int(point[0]), int(point[1])), 5, (0,255,0), -1)
    return

def visualize_bbox(bounding_boxes, img, labels=None):
    if labels is None:
        for point in bounding_boxes:
            cv2.circle(img, (int(point[0]), int(point[1])), 5, (0,0,255), -1)
    
    else:
        for point, labels in zip(bounding_boxes, labels):
            if labels == 0.0:
                cv2.circle(img, (int(point[0]), int(point[1])), 5, (255,100,0), -1)
            else:
                cv2.circle(img, (int(point[0]), int(point[1])), 5, (0,255,255), -1)
    return