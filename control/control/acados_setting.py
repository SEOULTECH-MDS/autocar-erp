#! /usr/bin/env python3

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from .bicycle_model import export_bicycle_modle
import numpy as np
from casadi import vertcat, SX, exp

def acados_solver():
    ocp = AcadosOcp()

    # 모델 가져오기
    model = export_bicycle_modle()

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
    MAX_SPEED = 2.0  # 최대 속도 [m/s]
    MIN_SPEED = -2.0  # 최소 속도 [m/s]
    
    # 상태 및 제어 입력 크기
    NX = model.x.size()[0]  # 상태 변수 크기
    NU = model.u.size()[0]  # 제어 입력 크기
    O = 2
    
    # 예측 시간 및 구간 설정
    T = 2.0  # 예측 시간 [s]
    N = 20  # 예측 구간 [s]
    
    # cost type 변경
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'

    # 비용 함수 가중치 설정 
    Q = np.diag([1.0, 1.0, 0.2, 0.1])  # 상태 변수 가중치 (x, y, yaw, v) 
    R = np.diag([0.1, 0.1])  # 제어 입력 가중치 (delta, v_cmd) 
    Rd = np.diag([1.0, 0.1])  # 제어 입력 변화량 가중치 (delta, v_cmd)
    Qe = np.diag([2.0, 2.0, 0.5, 0.1])  # 최종 상태 가중치 (x, y, yaw, v) 
    
    # 장애물 회피 가중치 (더 부드럽게 설정)
    W_obs = 50.0  # 장애물 회피 가중치 
    safe_distance_sq = 0.5  # 안전 거리 제곱 [m^2]
    barrier_gain = 2.0  # exponential barrier 게인
    
    # cost function 설정
    p = SX.sym('p', NX + NU + O)  # NX: 상태 변수 크기, NU: 제어 입력 크기, O: 장애물 정보 크기
    ocp.model.p = p
    ocp.parameter_values = np.zeros(NX + NU + O)

    # cost 수식에서 p를 사용
    x_err = ocp.model.x - ocp.model.p[:NX]  # 상태 오차
    u_prev = ocp.model.p[NX:NX+NU]  # 이전 제어 입력
    u_diff = ocp.model.u - u_prev  # 제어 입력 변화량
    obs_x = ocp.model.p[NX+NU]  # 장애물 x 좌표
    obs_y = ocp.model.p[NX+NU+1]  # 장애물 y 좌표

    # 차량 위치
    vehicle_x = ocp.model.x[0]  # 차량의 x 좌표
    vehicle_y = ocp.model.x[1]  # 차량의 y 좌표
    
    # 장애물과의 거리 제곱 계산
    distance_sq = (vehicle_x - obs_x)**2 + (vehicle_y - obs_y)**2
    
    # 부드러운 장애물 회피 비용 (exponential barrier function)
    # exp(-barrier_gain * (distance_sq - safe_distance_sq)) 형태로 부드러운 penalty
    # 거리가 가까울수록 exponentially 증가하는 비용
    obstacle_cost = W_obs * exp(-barrier_gain * (distance_sq - safe_distance_sq))

    # stage cost (장애물 회피 비용 포함)
    # stage_cost = (x_err.T @ Q @ x_err) + \
    #             (ocp.model.u.T @ R @ ocp.model.u) + \
    #             (u_diff.T @ Rd @ u_diff) + \
    #             obstacle_cost
    stage_cost = (x_err.T @ Q @ x_err) + \
                (ocp.model.u.T @ R @ ocp.model.u) + \
                (u_diff.T @ Rd @ u_diff) 
    ocp.model.cost_expr_ext_cost = stage_cost

    # terminal cost (장애물 회피 비용 포함)
    # terminal_cost = (x_err.T @ Qe @ x_err) + obstacle_cost
    terminal_cost = (x_err.T @ Qe @ x_err)
    ocp.model.cost_expr_ext_cost_e = terminal_cost

    # 제어 입력 제약 조건 (delta, v_cmd)
    ocp.constraints.lbu = np.array([-MAX_STEER, MIN_SPEED])  
    ocp.constraints.ubu = np.array([MAX_STEER, MAX_SPEED])   
    ocp.constraints.idxbu = np.array([0, 1])  # 제어 입력 인덱스

    # 상태 제약 조건 (x, y, yaw, v)
    ocp.constraints.x0 = np.zeros(NX) # 초기 상태
    ocp.constraints.lbx = np.array([-1e10, -1e10, -1e10, MIN_SPEED])  # 상태 변수 하한
    ocp.constraints.ubx = np.array([1e10, 1e10, 1e10, MAX_SPEED])    # 상태 변수 상한
    ocp.constraints.idxbx = np.array([0, 1, 2, 3])  # 모든 상태 변수에 대해 제약 조건 적용

    # Solver 옵션 설정 (최적화 문제 안정성 향상)
    ocp.solver_options.tf = T  # 예측 시간
    ocp.solver_options.N_horizon = N
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  
    ocp.solver_options.qp_solver_cond_N = 5  
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  
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
    ocp.solver_options.nlp_solver_step_length = 0.05  
    ocp.solver_options.levenberg_marquardt = 1e-4  

    solver = AcadosOcpSolver(ocp)  # Solver 생성

    return solver