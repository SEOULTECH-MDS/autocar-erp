#!/usr/bin/env python3
import numpy as np
import cv2

def get_cluster_2d(clusters_2d_msg):
    clusters_2d = []
    for cluster_msg in clusters_2d_msg.poses:
        x,y = cluster_msg.orientation.x, cluster_msg.orientation.y
        clusters_2d.append([x,y])
    return clusters_2d

def get_bbox(bboxes_msg):
    bboxes = []
    bbox_labels = []
    
    for bbox_msg in bboxes_msg.poses:
        x_min, y_min, x_max, y_max = bbox_msg.orientation.x, bbox_msg.orientation.y, bbox_msg.orientation.z, bbox_msg.orientation.w
        bbox_label = bbox_msg.position.x
        bboxes.append([x_min, y_min, x_max, y_max])
        bbox_labels.append(bbox_label)
        
    return bboxes, bbox_labels

def visualize_cluster_2d(clusters_2d, img):
    for point in clusters_2d:
        cv2.circle(img, (int(point[0]), int(point[1])), 5, (0,255,0), -1)
    return

def visualize_bbox(bounding_boxes, labels, img):
    for bbox, label in zip(bounding_boxes, labels):
        bbox = [int(e) for e in bbox]
        if label == 0.0:
            cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255,100,0), 2)
            # cv2.circle(img, (int(point[0]), int(point[1])), 5, (255,100,0), -1)
        else:
            cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,255), 2)
            # cv2.circle(img, (int(point[0]), int(point[1])), 5, (0,255,255), -1)
    return

def visualize_bbox_tracked(bounding_boxes, labels, img):
    for bbox, label in zip(bounding_boxes, labels):
        bbox = [int(e) for e in bbox]
        if label == 0.0:
            cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255,0,255), 1)
            # cv2.circle(img, (int(point[0]), int(point[1])), 5, (255,100,0), -1)
        else:
            cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,0), 1)
            # cv2.circle(img, (int(point[0]), int(point[1])), 5, (0,255,255), -1)
    return