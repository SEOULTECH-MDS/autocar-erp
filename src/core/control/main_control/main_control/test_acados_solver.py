#!/usr/bin/env python3

"""
Acados Solver Test Script
- solver 생성 테스트
- 기본적인 MPC 문제 해결 테스트
- 제약 조건 및 비용 함수 동작 확인
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from time import time
from .acados_setting import acados_solver


def test_acados_solver():

    solver = acados_solver()
    if solver is None:
        print("Solver creation failed.")
        return

    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    xref = np.zeros((4, 20))
    u_prev = np.zeros((2, 20))
    for i in range(20):
        xref[0, i] = 1.0 * i 
        xref[1, i] = 0.0
        xref[2, i] = 0.0
        xref[3, i] = 0.0

    solver.set(0, 'x', x0)
    solver.constraints_set(0, 'lbx', x0)
    solver.constraints_set(0, 'ubx', x0)

    obs = np.array([10.0, 0.0])


    status = solver.solve()
    if status != 0:
        print(f"Solver failed with status {status}.")
        return
    
    for i in range(20):
        u_prev[:, i] = np.array([solver.get(i, "u") for i in range(20)])







