#! /usr/bin/env python3

# mpc constraints setting function

import numpy as np
from casadi import vertcat

def constraints_settings(obstacle_positions=None, safe_distance=0.5):
    """
    제약 조건 설정 함수

    Args:
        obstacle_positions (list): 장애물의 위치 리스트 [(x1, y1), (x2, y2), ...].
        safe_distance (float): 장애물과의 최소 안전 거리 [m].

    Returns:
        dict: 제약 조건 설정 객체.
    """
    # 제약 조건 설정 객체
    constraints = {}

    # 제어 입력 제약 조건 (delta, v_cmd)
    MAX_STEER = np.deg2rad(30.0)  # 최대 조향각 [rad]
    MAX_SPEED = 1.5  # 최대 속도 [m/s]
    MIN_SPEED = -1.5  # 최소 속도 [m/s]

    constraints["lbu"] = np.array([-MAX_STEER, MIN_SPEED])  
    constraints["ubu"] = np.array([MAX_STEER, MAX_SPEED])   
    constraints["idxbu"] = np.array([0, 1])  # 제어 입력 인덱스

    # 상태 제약 조건 (x, y, yaw, v)
    constraints["lbx"] = np.array([MIN_SPEED])
    constraints["ubx"] = np.array([MAX_SPEED])
    constraints["idxbx"] = np.array([3])

    # 장애물 회피 제약 조건
    con_h_expr_list = []
    lg_list = []
    ug_list = []

    if obstacle_positions:
        for obs_x, obs_y in obstacle_positions:
            # 장애물 회피를 위한 거리 제약 조건
            con_h_expr = (vertcat(0)[0] - obs_x) ** 2 + (vertcat(0)[1] - obs_y) ** 2
            con_h_expr_list.append(con_h_expr)
            lg_list.append(safe_distance ** 2)  # 최소 안전 거리 제곱
            ug_list.append(np.inf)  # 상한 없음

    constraints["con_h_expr"] = vertcat(*con_h_expr_list) if con_h_expr_list else None
    constraints["lg"] = vertcat(*lg_list) if lg_list else None
    constraints["ug"] = vertcat(*ug_list) if ug_list else None

    return constraints
