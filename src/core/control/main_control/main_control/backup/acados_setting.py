#! /usr/bin/env python3

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from .bicycle_model import export_bicycle_model
import numpy as np
from casadi import SX, vertcat


def acados_solver():
    ocp = AcadosOcp()

    # 모델 가져오기
    model = export_bicycle_model()

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

    MAX_DSTEER = np.deg2rad(30.0)  # 최대 조향 각속도 [rad/s]
    MAX_ACCEL = 0.614  # 최대 가속도 [m/ss] defualt: 0.614
    
    # 상태 및 제어 입력 크기
    NX = 4  # 상태 변수 크기 (x, y, yaw, v)
    NU = 2  # 제어 입력 크기 (delta, v_cmd)
    O = 4 # 장애물 정보 크기 (obs1_x, obs1y, obs2_x, obs2_y)
    
    # 예측 시간 및 구간 설정
    T = 3.0  # 예측 시간 [s]
    N = 30  # 예측 구간 [s]
    
    # cost type 변경
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'

    # 비용 함수 가중치 설정 
    Q = np.diag([0.5, 0.5, 0.2, 0.0])  # 상태 변수 가중치 (x, y, yaw, v)  1.0 1.0 0.2 0.1
    R = np.diag([0.2, 0.2])  # 제어 입력 가중치 (delta, v_cmd) 0.1 0.1
    Rd = np.diag([0.1, 0.1])  # 제어 입력 변화량 가중치 (delta, v_cmd) 1.0 0.1
    Qe = np.diag([0.8, 0.8, 0.2, 0.0])  # 최종 상태 가중치 (x, y, yaw, v) 1.0 1.0 0.5 0.1
    
    # cost function 설정
    p = SX.sym('p', NX + NU + O)  # NX: 상태 변수 크기, NU: 제어 입력 크기, O: 장애물 정보 크기
    ocp.model.p = p
    ocp.parameter_values = np.zeros(NX + NU + O)

    # cost 수식에서 p를 사용
    x_err = ocp.model.x - ocp.model.p[:NX]  # 상태 오차
    u_prev = ocp.model.p[NX:NX+NU]  # 이전 제어 입력
    u_diff = ocp.model.u - u_prev  # 제어 입력 변화량
    obs1_x = ocp.model.p[NX+NU]  # 장애물1 x 좌표
    obs1_y = ocp.model.p[NX+NU+1]  # 장애물1 y 좌표
    obs2_x = ocp.model.p[NX+NU+2]  # 장애물2 x 좌표
    obs2_y = ocp.model.p[NX+NU+3]  # 장애물2 y 좌표

    # 차량 위치
    vehicle_x = ocp.model.x[0]  # 차량의 x 좌표
    vehicle_y = ocp.model.x[1]  # 차량의 y 좌표

    # stage cost 
    stage_cost = (x_err.T @ Q @ x_err) + \
                (ocp.model.u.T @ R @ ocp.model.u) + \
                (u_diff.T @ Rd @ u_diff) 
    ocp.model.cost_expr_ext_cost = stage_cost

    # terminal cost 
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

    # # 장애물 제약 조건
    # r_safe = 1.5
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

    # 코드 생성 경로 설정
    # 경로 꼬여서 노드 실행 안돼서 일단 주석처리
    # script_dir = os.path.dirname(os.path.abspath(__file__))
    # acados_path = os.path.join(script_dir, '..') # main_control 폴더
    # ocp.code_export_directory = os.path.join(acados_path)
    # ocp.json_file = os.path.join(acados_path)

    solver = AcadosOcpSolver(ocp)  # Solver 생성

    return solver