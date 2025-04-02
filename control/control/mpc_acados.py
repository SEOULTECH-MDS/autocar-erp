#!/usr/bin/env python3

from acados_template import AcadosOcp, AcadosOcpSolver
import casadi as ca
import numpy as np


# 상태 변수 (x, y, yaw, velocity)
x = ca.SX.sym('x')
y = ca.SX.sym('y')
yaw = ca.SX.sym('yaw')
v = ca.SX.sym('v')
states = ca.vertcat(x, y, yaw, v)

# 입력 변수 (steering angle, acceleration)
delta = ca.SX.sym('delta')
a = ca.SX.sym('a')
controls = ca.vertcat(delta, a)

# 파라미터 설정
L = 2.5  # Wheelbase
dt = 0.1 # Timestep

# 운동 방정식 정의
xdot = v * ca.cos(yaw)
ydot = v * ca.sin(yaw)
yawd = v * ca.tan(delta) / L
vdot = a

# 상태 변화율
f = ca.vertcat(xdot, ydot, yawd, vdot)

ocp = AcadosOcp()
ocp.model = f
ocp.dims.N = 20  # Prediction horizon (예측 시간 구간)

# 상태 및 입력 제약 조건 설정
ocp.constraints.lbu = np.array([-0.5, -2])  # 조향각, 가속도 최소값
ocp.constraints.ubu = np.array([0.5, 2])    # 조향각, 가속도 최대값
ocp.constraints.x0 = np.array([0, 0, 0, 0]) # 초기 상태
ocp.cost.cost_type = "LINEAR_LS"

# Solver 설정
ocp.solver_options.qp_solver = "HPIPM"
ocp.solver_options.integrator_type = "ERK"

# Solver 생성
ocp_solver = AcadosOcpSolver(ocp, json_file="mpc_acados.json")
