#! /usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import math

class FrenetPath(object):
    def __init__(self, si, di, ddi, dddi, sf, df, ddf, dddf):
        A = np.array([[si**5,    si**4,    si**3,   si**2, si,  1.0],
                      [5*si**4,  4*si**3,  3*si**2, 2*si,  1.0, 0.0],
                      [20*si**3, 12*si**2, 6*si,    2.0,   0.0, 0.0],
                      [sf**5,    sf**4,    sf**3,   sf**2, sf,  1.0],
                      [5*sf**4,  4*sf**3,  3*sf**2, 2*sf,  1.0, 0.0],
                      [20*sf**3, 12*sf**2, 6*sf,    2.0,   0.0, 0.0]])

        b = np.array([di, ddi, dddi, df, ddf, dddf])

        x = np.linalg.solve(A, b)

        self.p = np.poly1d(x)
        self.dp = self.p.deriv()
        self.ddp = self.dp.deriv()
        self.target_d = df
        self.kappa = 0

        self.si = si
        self.sf = sf
        self.x = []
        self.y = []
        self.ds = []
        self.yaw = []

        self.s = []
        self.d = []
        self.gap = 100.0

class Frenet(object):
    def __init__(self, ref_path, car_pose, robot_radius, lane_width, possible_change_direction):
        self.ref_path = ref_path
        # s, d, yaw_road = self.get_frenet(x, y)
        # self.prev_opt_path = FrenetPath(s, d, np.tan(yaw-yaw_road), 0, s+1, 0, 0, 0)
        self.robot_radius = robot_radius
        self.LANE_WIDTH = lane_width
        self.is_avoiding = False

        start_point, _ = self.find_closest_wp(car_pose, self.ref_path)
        
        s, d, yaw_road = self.get_frenet(start_point, self.ref_path)
        self.prev_opt_path = FrenetPath(s, d, np.tan(car_pose[2] - yaw_road), 0, s+1, 0, 0, 0)
        
        # 3,6 / 4,12
        self.MIN_SF = 4.0
        self.DS = 6.0 # 종방향으로 얼마나 쪼갤지
        self.steps = 20

        # cost weights 1001
        self.K = 3 # 원래 ref path로 돌아오려는 weight
        self.K_DIFF = 3
        self.K_KAPPA = 0
        self.K_MEAN_S = 0
        self.K_SHORT = 0
        self.K_COLLISION = 100

        if possible_change_direction == 'right':
            self.DF_SET = np.array([-self.LANE_WIDTH, 0.0])
            self.ob_radius = 0.7
            self.MAX_SF = 20.0
            self.MIN_SF = 4.0
            self.DS = 6.0

        elif possible_change_direction == 'left':
            # self.DF_SET = np.array([-0.3 * self.LANE_WIDTH, 0.0, self.LANE_WIDTH])
            self.DF_SET = np.array([-0.1 * self.LANE_WIDTH, 0.0, self.LANE_WIDTH * 0.8, self.LANE_WIDTH*1.2])
            self.ob_radius = 0.7
            self.MAX_SF = 20.0
            self.MIN_SF = 3.0
            self.DS = 6.0

        elif possible_change_direction == 'both':
            self.DF_SET = np.array([-self.LANE_WIDTH, 0.0, self.LANE_WIDTH])
            self.ob_radius = 0.7
            self.MAX_SF = 20.0
            self.MIN_SF = 4.0
            self.DS = 6.0

        else: # 같은 차선 내
            self.DF_SET = np.array([-self.LANE_WIDTH * 0.25, -self.LANE_WIDTH * 0.15, 0.0, 
                                   self.LANE_WIDTH * 0.15, self.LANE_WIDTH * 0.25])
                                    #   self.LANE_WIDTH * 0.2, self.LANE_WIDTH * 0.3, self.LANE_WIDTH * 0.35])
            self.DDF_SET = [-0.5, -0.3, 0.0, 0.3, 0.5]
            # self.DDF_SET = [-0.5, -0.3, 0.0, 0.3, 0.5]
            self.ob_radius = 0.35
            self.MAX_SF = 14.5 # 길이
            self.MIN_SF = 0.5 # 곡률
            self.DS = 2.0
            
            # self.DF_SET = np.array([-self.LANE_WIDTH * 0.1, 0.0, 
            #                        self.LANE_WIDTH * 0.2,  self.LANE_WIDTH * 0.35])
            # self.MAX_SF = 4.0 # 길이
            # self.MIN_SF = 1.0 # 곡률
            # self.DS = 3.0

    def get_frenet(self, cur_position, ref_path):
        _, idx = self.find_closest_wp(cur_position, ref_path)

        next_wp = idx + 1
        prev_wp = idx

        n_x = ref_path['x'][next_wp] - ref_path['x'][prev_wp]
        n_y = ref_path['y'][next_wp] - ref_path['y'][prev_wp]
        ego_vec = [cur_position[0] - ref_path['x'][prev_wp], cur_position[1] - ref_path['y'][prev_wp]]
        map_vec = [n_x, n_y]

        d= np.cross(map_vec, ego_vec)/(ref_path['s'][next_wp] - ref_path['s'][prev_wp])
        s = ref_path['s'][prev_wp] + np.dot(map_vec, ego_vec)/(ref_path['s'][next_wp] - ref_path['s'][prev_wp])
        heading = np.arctan2(n_y, n_x)

        return s, d, heading
    
    def get_cartesian(self, s, d, ref_path):
        prev_wp = np.searchsorted(ref_path['s'], s, 'left') - 1

        heading = ref_path['yaw'][prev_wp]
        perp_heading = heading + np.deg2rad(90)

        seg_s = s - ref_path['s'][prev_wp]
        x = ref_path['x'][prev_wp] + seg_s*np.cos(heading) + d*np.cos(perp_heading)
        y = ref_path['y'][prev_wp] + seg_s*np.sin(heading) + d*np.sin(perp_heading)

        return x, y, heading

    def find_path(self, car_pose, obstacles):
        # car_pos: 차량 (x,y,yaw)
        # ref_path: 경로 정보 (xs, ys, yaws)
        # obstacles: utm frame 상에서 장애물 중심 좌표, 크기(반경)

        frenet_paths, car_d = self.calc_frenet_paths(car_pose, self.ref_path)
        frenet_paths = self.calc_global_paths(frenet_paths, self.ref_path)
        frenet_paths = self.check_path(frenet_paths, obstacles)

        min_cost = float("inf")
        opt_path = None

        for fp in frenet_paths:
            # cost for origin path (last)
            d = abs(fp.d[-1])
            
            # cost for consistency
            # d_diff = abs(self.prev_opt_path.target_d - fp.target_d)

            # cost for path length
            path_sum = np.sum(fp.ds)

            # cost for curvature
            yaw_diff = fp.yaw[1:] - fp.yaw[:-1]
            yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            yaw_diff = np.abs(yaw_diff)

            mean_kappa = np.sum(yaw_diff / fp.ds[:-1])
            fp.kappa = np.sum(abs(fp.d))/len(fp.d)
                
            kappa_diff = fp.kappa - self.prev_opt_path.kappa

            # cost
            fp.c = self.K * d + self.K_DIFF * kappa_diff \
                    + self.K_SHORT * path_sum \
                    + self.K_KAPPA * mean_kappa \
                    + self.K_MEAN_S * fp.kappa \
                    + self.K_COLLISION / (fp.gap + 0.01)
            print('cost:', fp.d[-1], ',', fp.gap, ',' ,fp.c)
            if min_cost > fp.c:
                min_cost = fp.c
                opt_path = fp
                    
        if opt_path is not None:
            self.prev_opt_path = opt_path
            self.update_avoidance_status(opt_path, car_d)
        # else:
        #     opt_path = self.prev_opt_path
        #     # opt path가 없으면 이전 opt path를 활용
        #     # 이전 opt path의 s값을 offset 시켜서 쏴주면 더 좋을거같음

        # elif len(opt_path.x) == 0:
        else:
            if len(self.prev_opt_path.x) == 0:
                normal_path = self.find_normal_path(car_pose)
                return frenet_paths, [normal_path['x'], normal_path['y'], normal_path['yaw']], False, 0, False
            else:
                opt_path = self.prev_opt_path

        
        # print("opt path: ", opt_path)
        return frenet_paths, [opt_path.x, opt_path.y, opt_path.yaw], self.is_avoiding, 0, False

    def calc_frenet_paths(self, car_pose, ref_path):
        # path initial conidion
        si, car_d, _ = self.get_frenet(car_pose, ref_path)
        di = self.prev_opt_path.p(si)
        ddi = self.prev_opt_path.dp(si)
        dddi = self.prev_opt_path.ddp(si)
        # ddi = -0.05
        #path final condition
        ddf = 0.0
        dddf = 0.0

        # frenet_paths = []
        # for df in self.DF_SET:
        #     for sf in np.arange(si+self.MIN_SF, si+self.MAX_SF + self.DS, self.DS):
        #         fp = FrenetPath(si, di, ddi, dddi, sf, df, ddf, dddf)
        #         frenet_paths.append(fp)

        frenet_paths = []
        for df in self.DF_SET:
            for sf in np.arange(si+self.MIN_SF, si+self.MAX_SF, self.DS):
                # for ddf in self.DDF_SET:  # 방향 전환을 위한 가속도 변화
                    # for ddi in [0, -0.05]:
                        fp = FrenetPath(si, di, ddi, dddi, sf, df, ddf, dddf)
                        frenet_paths.append(fp)

        
        return frenet_paths, car_d

    def calc_global_paths(self, fplist, ref_path):
        for fp in fplist:
            fp.s = np.linspace(fp.si, fp.si + self.MAX_SF, self.steps)
            fp.d = np.where(fp.s < fp.sf, fp.p(fp.s), fp.target_d)

            for _s, _d in zip(fp.s, fp.d):
                _x, _y, _ = self.get_cartesian(_s, _d, ref_path)
                fp.x.append(_x)
                fp.y.append(_y)
            fp.x = np.array(fp.x)
            fp.y = np.array(fp.y)

            dx = fp.x[1:] - fp.x[:-1]
            dy = fp.y[1:] - fp.y[:-1]
            fp.yaw = np.arctan2(dy, dx)
            fp.ds = np.hypot(dx, dy)
            fp.yaw = np.append(fp.yaw, fp.yaw[-1])
            fp.ds = np.append(fp.ds, fp.ds[-1])

        return fplist

    def collision_check(self, fp, obstacles):
        for ob_x, ob_y, ob_radius_ in obstacles:
            d = [((_x - ob_x) ** 2 + (_y - ob_y) ** 2) for (_x, _y) in zip(fp.x, fp.y)]
            
            ob_radius = min(self.ob_radius, ob_radius_)
            # ob_radius = self.ob_radius
            collision_dist = np.sqrt(min(d)) - self.robot_radius - ob_radius
            # fp.gap = np.sqrt(min(d))
            
            # print(np.sqrt(min(d)), ob_radius)
            collision = collision_dist <= 0

            if collision:
                return True
        return False

    def check_path(self, fplist, obstacles):
        ok_ind = []
        for i, _path in enumerate(fplist):
            if self.collision_check(_path, obstacles):
                continue
            ok_ind.append(i)

        if not ok_ind:
            print("No Path Found")
        return [fplist[i] for i in ok_ind]
    
    def find_closest_wp(self, car_pose, ref_path):
        idx = None
        min_dist = np.inf
        for i, (path_x, path_y) in enumerate(zip(ref_path['x'], ref_path['y'])):
            dist = (path_x - car_pose[0])**2 + (path_y - car_pose[1])**2
            if dist < min_dist:
                min_dist = dist
                idx = i

        idx = min(idx, len(ref_path['x'])-2)
        closest_wp = (ref_path['x'][idx], ref_path['y'][idx])
        return closest_wp, idx
    
    def update_avoidance_status(self, path, car_d):
        # path.d[-1] 절대값이 일정 이하 & 현재 경로 상에서 차량 d 값이 일정 이하
        # => 정상 주행

        kappa = path.kappa

        if kappa > 0.25:
            self.is_avoiding = True
            # print('장애물 cost:', path.gap)
        
        if self.is_avoiding == True and abs(car_d) < 0.3 and kappa <= 0.5:
            self.is_avoiding = False

        return 
    
    def find_normal_path(self, car_pose):
        car_x, car_y = car_pose[0], car_pose[1]
        
        # _, cur_idx = self.path_kdtree.query([car_x, car_y])
        cur_idx = None
        min_dist = np.inf
        for idx, (path_x_, path_y_) in enumerate(zip(self.ref_path['x'], self.ref_path['y'])):
            dist = (path_x_ - car_x)**2 + (path_y_ - car_y)**2
            if dist < min_dist:
                cur_idx = idx
                min_dist = dist
        
        # 가장 가까운 노드로부터 앞으로 10개, 뒤로 4개 찾기
        n_back = 4
        n_forward = 10
        path_len = len(self.ref_path['x'])

        start_index = max(cur_idx - n_back, 0)
        end_index = min(cur_idx + n_forward, path_len)
        
        # 인근 waypoints 추출
        path = {'x':[], 'y':[], 'yaw':[]}
        path['x'] = self.ref_path['x'][start_index:end_index + 1]
        path['y'] = self.ref_path['y'][start_index:end_index + 1]
        path['yaw'] = self.ref_path['yaw'][start_index:end_index + 1]
        return path
    
    # # Atsushi Sakai 코드 복붙
    # def calc_global_paths(self, fplist, ref_path):
    #     for fp in fplist:
    #         # print("==============================================")
    #         # print("==============================================")
    #         # print("==============================================")
    #         # for _s, _d in zip(fp.s, fp.d):
    #         #     _x, _y, _ = self.get_cartesian(_s, _d, ref_path)
    #         #     fp.x.append(_x)
    #         #     fp.y.append(_y)
    #         # fp.x = np.array(fp.x)
    #         # fp.y = np.array(fp.y)
    #         # dx = fp.x[1:] - fp.x[:-1]
    #         # dy = fp.y[1:] - fp.y[:-1]
    #         # fp.yaw = np.arctan2(dy, dx)
    #         # fp.ds = np.hypot(dx, dy)
    #         # fp.yaw = np.append(fp.yaw, fp.yaw[-1])
    #         # fp.ds = np.append(fp.ds, fp.ds[-1])
    #         # print("==============================================")

    #         # calc global positions
    #         for i in range(len(fp.s)):
    #             ix, iy = ref_path['csp'].calc_position(fp.s[i])
    #             if ix is None:
    #                 break
    #             i_yaw = ref_path['csp'].calc_yaw(fp.s[i])
    #             di = fp.d[i]
    #             fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
    #             fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
    #             fp.x.append(fx)
    #             fp.y.append(fy)

    #         # calc yaw and ds
    #         for i in range(len(fp.x) - 1):
    #             dx = fp.x[i + 1] - fp.x[i]
    #             dy = fp.y[i + 1] - fp.y[i]
    #             fp.yaw.append(math.atan2(dy, dx))
    #             fp.ds.append(math.hypot(dx, dy))

    #         fp.yaw.append(fp.yaw[-1])
    #         fp.ds.append(fp.ds[-1])

    #         # calc curvature
    #         for i in range(len(fp.yaw) - 1):
    #             fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    #     return fplist

