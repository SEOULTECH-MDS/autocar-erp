#! /usr/bin/env python3

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from .bicycle_model_sp import export_spline_bicycle_model
import numpy as np
from casadi import SX, vertcat

def acados_solver():
    ocp = AcadosOcp()
    
    # 모델 가져오기
    model = export_spline_bicycle_model()

    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.name = model.name
    ocp.model = model_ac

    # 제약 조건 설정
    MAX_STEER = np.deg2rad(30.0)  # 최대 조향각 [rad]
    MAX_SPEED = 4.0  # 최대 속도 [m/s]
    MIN_SPEED = -4.0  # 최소 속도 [m/s] 
    MAX_ACCEL = 10.0  # 최대 가속도 [m/s^2] (마찰력 극복을 위해 증가)
    MIN_ACCEL = -1e3  # 최대 감속도 [m/s^2]

    NX = 5  # reference size (x, y, yaw, v, s)
    ND = 1 # 이전 조향각 입력 크기 (delta)
    NV = 2 # tangent vector size (tx, ty)
    NO = 4  # 장애물 정보 크기 (obs1_x, obs1_y, obs2_x, obs2_y)
    
    T = 3.0
    N = 30  # 예측 구간 [s]

    # cost type
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'

    # cost function weights (TMPC weights)
    # W_acc = 0.34  # 가속도 입력 크기 가중치 0.1
    # W_steer = 0.85  # 조향각 입력 크기 가중치 0.2
    # W_v = 0.55  # 속도 error 가중치 0.1
    # W_lag = 0.75  # lag error 가중치 1.0
    # W_con = 0.05  # contour error 가중치 1.0
    # W_yaw = 0.5  # heading error 가중치 (terminal cost) 0.5
    # m_term = -0.4  # terminal cost에서 lag, con error 비율 감소시키는 가중치

    # cost function weights (0819 test)
    # W_acc = 0.1  # 가속도 입력 크기 가중치 0.1
    # W_steer = 0.85  # 조향각 입력 크기 가중치 0.2
    # W_v = 0.1  # 속도 error 가중치 0.1
    # W_lag = 1.0  # lag error 가중치 1.0
    # W_con = 0.3  # contour error 가중치 1.0
    # W_yaw = 0.3  # heading error 가중치 (terminal cost) 0.5
    # m_term = -0.3 # terminal cost에서 lag, con error 비율 감소시키는 가중치

    # cost function weights 
    W_acc = 0.1  # 가속도 입력 크기 가중치 (대폭 감소 - 빠른 가속 적극 허용)
    W_steer = 0.2  # 조향각 입력 크기 가중치 0.2
    W_steer_rate = 1.5  # 조향각 변화율 가중치
    W_v = 0.5  # 속도 error 가중치 
    W_lag = 1.0  # lag error 가중치 1.0
    W_con = 0.1  # contour error 가중치 1.0

    We_v = W_v * 2.0  # terminal cost에서 속도 error 가중치
    We_lag = W_lag * 3.0  # terminal cost에서 lag error 가중치
    We_con = W_con  # terminal cost에서 contour error 가중치
    We_yaw = 0.4  # heading error 가중치 (terminal cost)

    # parameter variables
    NP = NX + ND + NV + NO   # NX: 참조 변수 크기, ND: 이전 조향각 입력 크기 NV: 접선 벡터 크기, O: 장애물 정보 크기
    p = SX.sym('p', NP) 
    ocp.model.p = p
    ocp.parameter_values = np.zeros(NP)

    x_ref = ocp.model.p[0]  # ref x 좌표
    y_ref = ocp.model.p[1]  # ref y 좌표
    yaw_ref = ocp.model.p[2]  # ref yaw 각도
    v_ref = ocp.model.p[3]  # ref 속도
    s_ref = ocp.model.p[4]  # ref spline parameter (사용X)
    prev_steering_angle = ocp.model.p[5] # 이전 조향각 입력
    tx = ocp.model.p[6]  # 접선 벡터 x
    ty = ocp.model.p[7]  # 접선 벡터 y
    obs1_x = ocp.model.p[8]  # 장애물1 x 좌표
    obs1_y = ocp.model.p[9]  # 장애물1 y 좌표
    obs2_x = ocp.model.p[10]  # 장애물2 x 좌표
    obs2_y = ocp.model.p[11]  # 장애물2 y 좌표

    vehicle_x = ocp.model.x[0]  # 차량 x 좌표
    vehicle_y = ocp.model.x[1]  # 차량 y 좌표
    vehicle_yaw = ocp.model.x[2]  # 차량 yaw 각도
    vehicle_v = ocp.model.x[3]  # 차량 속도

    steering_angle = ocp.model.u[0]  # 조향각
    acceleration = ocp.model.u[1]  # 가속도

    # stage cost function
    stage_cost = W_acc * (acceleration ** 2) + \
                 W_steer * (steering_angle ** 2) + \
                 W_steer_rate * (steering_angle - prev_steering_angle) ** 2 + \
                 W_v * ((vehicle_v - v_ref) ** 2) + \
                 W_lag * ((tx*(vehicle_x - x_ref) + ty*(vehicle_y - y_ref)))**2 + \
                 W_con * ((ty*(vehicle_x - x_ref) - tx*(vehicle_y - y_ref)))**2
    ocp.model.cost_expr_ext_cost = stage_cost

    # terminal cost function
    terminal_cost = We_v * ((vehicle_v - v_ref) ** 2) + \
                    We_lag * ((tx*(vehicle_x - x_ref) + ty*(vehicle_y - y_ref)))**2 + \
                    We_con * ((ty*(vehicle_x - x_ref) - tx*(vehicle_y - y_ref)))**2 + \
                    We_yaw * ((vehicle_yaw - yaw_ref) ** 2)                    
    ocp.model.cost_expr_ext_cost_e = terminal_cost

    # constraints
    ocp.constraints.lbu = np.array([-MAX_STEER, MIN_ACCEL])
    ocp.constraints.ubu = np.array([MAX_STEER, MAX_ACCEL])
    ocp.constraints.idxbu = np.array([0, 1])  # 제어 입력 인덱스

    ocp.constraints.x0 = np.zeros(NX)  # 초기 상태
    ocp.constraints.lbx = np.array([-1e10, -1e10, -1e10, MIN_SPEED, -1e10])  # 상태 변수 하한
    ocp.constraints.ubx = np.array([1e10, 1e10, 1e10, MAX_SPEED, 1e10])  # 상태 변수 상한
    ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4]) # 상태 변수 인덱스

    # r_safe = 1.0
    # distance1 = (vehicle_x - obs1_x)**2 + (vehicle_y - obs1_y)**2
    # distance2 = (vehicle_x - obs2_x)**2 + (vehicle_y - obs2_y)**2
    # ocp.model.con_h_expr = vertcat(distance1, distance2)
    # ocp.constraints.lh = np.array([r_safe**2, r_safe**2])  
    # ocp.constraints.uh = np.array([1e10, 1e10]) 

    # Solver 옵션 설정 
    ocp.solver_options.tf = T  # 예측 시간
    ocp.solver_options.N_horizon = N
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  
    ocp.solver_options.qp_solver_cond_N = 5  
    ocp.solver_options.hessian_approx = "EXACT"  
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 200  
    ocp.solver_options.qp_solver_iter_max = 100  
    ocp.solver_options.nlp_solver_tol_stat = 1e-4  
    ocp.solver_options.nlp_solver_tol_eq = 1e-4    
    ocp.solver_options.nlp_solver_tol_ineq = 1e-4  
    ocp.solver_options.nlp_solver_tol_comp = 1e-4  
    ocp.solver_options.globalization = "MERIT_BACKTRACKING"  
    ocp.solver_options.regularize_method = "CONVEXIFY"  
    ocp.solver_options.globalization_fixed_step_length = 0.05
    ocp.solver_options.levenberg_marquardt = 1e-4  

    solver = AcadosOcpSolver(ocp)

    return solver