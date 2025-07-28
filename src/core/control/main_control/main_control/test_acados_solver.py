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
from acados_setting import acados_solver


def test_acados_solver():

    solver = acados_solver()
    if solver is None:
        print("Solver creation failed.")
        return
    
    x = 0.0
    y = 0.0
    yaw = 0.0
    v = 0.0
    delta = 0.0
    v_cmd = 0.0

    x0 = np.array([0.0, 0.0, 0.0, 4.0])
    obs = np.array([10.0, 10.0])
    
    xref = np.zeros((4, 20))
    u_prev = np.zeros((2, 20))
    for i in range(20):
        xref[0, i] = 1.0 * i 
        xref[1, i] = 1.0 * i 
        xref[2, i] = 0.0
        xref[3, i] = 0.0

    solver.set(0, 'x', x0)
    solver.constraints_set(0, 'lbx', x0)
    solver.constraints_set(0, 'ubx', x0)

    for i in range(20):
        solver.set(i, "p", np.hstack([xref[:, i], u_prev[:,i], obs]))
    solver.set(20, "p", np.hstack([xref[:, -1], u_prev[:, -1], obs]))

    status = solver.solve()
    if status != 0:
        print(f"Solver failed with status {status}.")
        return
    
    x_traj = np.zeros((4, 20))
    u_traj = np.zeros((2, 20))
    for i in range(20):
        x_traj[:, i] = solver.get(i, 'x')
        u_traj[:, i] = solver.get(i, 'u')

    print("x trajectory:", x_traj)
    print("u trajectory:", u_traj)
    
    # 시각화
    plt.figure(figsize=(12, 8))
    
    # 궤적 플롯
    plt.subplot(2, 2, 1)
    plt.plot(x_traj[0, :], x_traj[1, :], 'b-o', linewidth=2, markersize=4, label='Vehicle Trajectory')
    plt.plot(xref[0, :], xref[1, :], 'r--', linewidth=2, label='Reference Trajectory')
    plt.scatter(obs[0], obs[1], c='red', s=100, marker='x', linewidth=3, label='Obstacle')
    plt.xlabel('X position [m]')
    plt.ylabel('Y position [m]')
    plt.title('Vehicle Trajectory and Obstacle')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # 속도 프로필
    plt.subplot(2, 2, 2)
    plt.plot(range(20), x_traj[3, :], 'b-o', linewidth=2, markersize=4, label='Vehicle Velocity')
    plt.plot(range(20), xref[3, :], 'r--', linewidth=2, label='Reference Velocity')
    plt.xlabel('Time step')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity Profile')
    plt.legend()
    plt.grid(True)
    
    # 조향각 프로필
    plt.subplot(2, 2, 3)
    plt.plot(range(20), u_traj[0, :], 'g-o', linewidth=2, markersize=4, label='Steering Angle')
    plt.plot(range(20), x_traj[2, :], 'y--', linewidth=2, label='Yaw Angle')
    plt.xlabel('Time step')
    plt.ylabel('Steering Angle [rad]')
    plt.title('Steering Control Input')
    plt.legend()
    plt.grid(True)
    
    # 가속도 프로필
    plt.subplot(2, 2, 4)
    plt.plot(range(20), u_traj[1, :], 'm-o', linewidth=2, markersize=4, label='Acceleration')
    plt.xlabel('Time step')
    plt.ylabel('Acceleration [m/s²]')
    plt.title('Acceleration Control Input')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()



test_acados_solver()
    


    







