#! /usr/bin/env python3

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from bicycle_model import export_bicycle_modle
import numpy as np
from casadi import vertcat

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
    MAX_SPEED = 4.0  # 최대 속도 [m/s]
    MIN_SPEED = -4.0  # 최소 속도 [m/s]

    # 제어 입력 제약 조건 (delta, v_cmd)
    ocp.constraints.lbu = np.array([-MAX_STEER, MIN_SPEED])  
    ocp.constraints.ubu = np.array([MAX_STEER, MAX_SPEED])   
    ocp.constraints.idxbu = np.array([0, 1])  # 제어 입력 인덱스

    # 상태 제약 조건 (x, y, yaw, v)
    ocp.constraints.lbx = np.array([-1e10, -1e10, -1e10, MIN_SPEED])  # 상태 변수 하한
    ocp.constraints.ubx = np.array([1e10, 1e10, 1e10, MAX_SPEED])    # 상태 변수 상한
    ocp.constraints.idxbx = np.array([0, 1, 2, 3])  # 모든 상태 변수에 대해 제약 조건 적용

    # 디버깅 출력
    print("lbx:", ocp.constraints.lbx)
    print("ubx:", ocp.constraints.ubx)
    print("idxbx:", ocp.constraints.idxbx)

    print("lbu:", ocp.constraints.lbu)
    print("ubu:", ocp.constraints.ubu)
    print("idxbu:", ocp.constraints.idxbu)

    # 장애물 회피 제약 조건 (필요 시 추가)
    obstacle_positions = None  # 장애물 위치 리스트 [(x1, y1), (x2, y2), ...]
    safe_distance = 0.5  # 장애물과의 최소 안전 거리 [m]
    if obstacle_positions:
        con_h_expr_list = []
        lg_list = []
        ug_list = []
        for obs_x, obs_y in obstacle_positions:
            # 장애물 회피를 위한 거리 제약 조건
            con_h_expr = (vertcat(0)[0] - obs_x) ** 2 + (vertcat(0)[1] - obs_y) ** 2
            con_h_expr_list.append(con_h_expr)
            lg_list.append(safe_distance ** 2)  # 최소 안전 거리 제곱
            ug_list.append(np.inf)  # 상한 없음

        ocp.constraints.con_h_expr = vertcat(*con_h_expr_list) if con_h_expr_list else None
        ocp.constraints.lg = vertcat(*lg_list) if lg_list else None
        ocp.constraints.ug = vertcat(*ug_list) if ug_list else None

    # 상태 및 제어 입력 크기
    NX = model.x.size()[0]  # 상태 변수 크기
    NU = model.u.size()[0]  # 제어 입력 크기
    NY = NX + NU  # 참조 값 크기

    # 예측 시간 및 구간 설정
    T = 2.0  # 예측 시간 [s]
    N = 20  # 예측 구간 [s]

    # 비용 함수 가중치 설정
    Q = np.diag([0.5, 0.5, 0.5, 0.1])  # 상태 변수 가중치 (x, y, yaw, v)
    R = np.diag([0.1, 0.01])  # 제어 입력 가중치 (delta, v_cmd)
    Qe = np.diag([0.5, 0.5, 0.5, 0.1])  # 최종 상태 가중치 (x, y, yaw, v)

    ocp.cost.W = np.block([
        [Q, np.zeros((NX, NU))],
        [np.zeros((NU, NX)), R]
    ])
    ocp.cost.W_e = Qe  # 최종 상태 가중치

    # 참조 값 설정
    ocp.cost.yref = np.zeros(NY)  # 상태 + 제어 입력 참조 값
    ocp.cost.yref_e = np.zeros(NX)  # 최종 상태 참조 값

    # Vx, Vu, Vx_e 설정
    ocp.cost.Vx = np.block([
        [np.eye(NX)],  # 상태 변수 선택
        [np.zeros((NU, NX))]  # 제어 입력은 상태 변수 선택에 포함되지 않음
    ])
    ocp.cost.Vu = np.block([
        [np.zeros((NX, NU))],  # 상태 변수는 제어 입력 선택에 포함되지 않음
        [np.eye(NU)]  # 제어 입력 선택
    ])
    ocp.cost.Vx_e = np.eye(NX)  # 최종 상태 선택 행렬 (모든 상태 변수를 포함)

    # Solver 옵션 설정
    ocp.solver_options.tf = T  # 예측 시간
    ocp.dims.N = N

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 200

    solver = AcadosOcpSolver(ocp)  # Solver 생성

    return solver