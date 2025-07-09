#! /usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
from scipy.spatial import KDTree

class Normal():
    def __init__(self, ref_path, car_pose):
        self.ref_path = ref_path

        path_coords = np.vstack((self.ref_path['x'], self.ref_path['y'])).T
        self.path_kdtree = KDTree(path_coords)

    def find_path(self, car_pose, obstacles):
        car_x, car_y = car_pose[0], car_pose[1]
        
        _, cur_idx = self.path_kdtree.query([car_x, car_y])
        # cur_idx = None
        # min_dist = np.inf
        # for idx, (path_x_, path_y_) in enumerate(zip(self.ref_path['x'], self.ref_path['y'])):
        #     dist = (path_x_ - car_x)**2 + (path_y_ - car_y)**2
        #     if dist < min_dist:
        #         cur_idx = idx
        #         min_dist = dist
        
        # 가장 가까운 노드로부터 앞으로 10개, 뒤로 4개 찾기
        n_back = 4
        n_forward = 10
        path_len = len(self.ref_path['x'])

        start_index = max(cur_idx - n_back, 0)
        end_index = min(cur_idx + n_forward, path_len)
        
        # 인근 waypoints 추출
        path_xs = self.ref_path['x'][start_index:end_index + 1]
        path_ys = self.ref_path['y'][start_index:end_index + 1]
        path_yaws = self.ref_path['yaw'][start_index:end_index + 1]
        return [], [path_xs, path_ys, path_yaws], False, 0, False
    