import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from casadi import SX, vertcat
import math

# 차량 모델 정의
def vehicle_model():
    x = SX.sym('x')      # 차량의 x 위치
    y = SX.sym('y')      # 차량의 y 위치
    yaw = SX.sym('yaw')  # 차량의 yaw 각도
    v = SX.sym('v')      # 차량의 속도

    delta = SX.sym('delta')  # 조향각
    v_cmd = SX.sym('v_cmd')  # 목표 속도 (제어 입력)

    # 상태 및 제어 입력
    states = vertcat(x, y, yaw, v)
    controls = vertcat(delta, v_cmd)

    # 차량 모델 방정식
    L = 1.566  # 차량의 휠베이스 (ERP42: 1.566)
    dx = v * np.cos(yaw)
    dy = v * np.sin(yaw)
    dyaw = v * np.tan(delta) / L
    dv = (v_cmd - v) / 0.5  # 속도 변화율 (0.5초의 시간 상수 사용)

    # 상태 변화율
    f = vertcat(dx, dy, dyaw, dv)

    return states, controls, f

# MPC 설정
def create_ocp():
    ocp = AcadosOcp()

    # 차량 모델 정의
    states, controls, f = vehicle_model()
    nx = states.size()[0]  # 상태 변수 개수 (4: [x, y, yaw, v])
    nu = controls.size()[0]  # 제어 입력 개수 (2: [delta, v_cmd])

    # OCP 모델 설정
    ocp.model.name = "vehicle_mpc"
    ocp.model.x = states
    ocp.model.u = controls
    ocp.model.f_expl_expr = f
    ocp.model.f_impl_expr = f - states

    # 시간 설정
    T = 5  # 예측 시간 (초)
    N = 20  # 예측 단계 수
    ocp.dims.N = N
    ocp.solver_options.tf = T

    # 비용 함수 설정
    Q = np.diag([1.2, 1.2, 0.5, 0.5])  # 상태 비용 행렬 (4x4)
    R = np.diag([0.01, 0.05])          # 제어 입력 비용 행렬 (2x2)
    ny = nx + nu  # 비용 함수의 총 차원 (상태 + 제어 입력)

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])  # (6x6) 비용 행렬
    ocp.cost.W_e = Q  # 최종 상태 비용 행렬 (4x4)

    # 비용 함수 표현식 정의
    ocp.model.cost_y_expr = vertcat(states, controls)  # [x, y, yaw, v, delta, v_cmd]
    ocp.model.cost_y_expr_e = states  # [x, y, yaw, v]

    # 참조 값 초기화
    x_ref = np.zeros(nx)  # 상태 참조 값 (4,)
    u_ref = np.zeros(nu)  # 제어 입력 참조 값 (2,)
    ocp.cost.yref = np.concatenate((x_ref, u_ref))  # (6,)
    ocp.cost.yref_e = x_ref  # (4,)

    # 제약 조건 설정
    ocp.constraints.lbu = np.array([-np.deg2rad(30), 0.0])  # 최소 제어 입력 (조향각, 속도)
    ocp.constraints.ubu = np.array([np.deg2rad(30), 1.5])   # 최대 제어 입력 (조향각, 속도)
    ocp.constraints.idxbu = np.array([0, 1])  # 제어 입력 인덱스
    ocp.constraints.lbx = np.array([-np.inf, -np.inf, -np.inf, 0.0])  # 최소 상태 (속도는 0 이상)
    ocp.constraints.ubx = np.array([np.inf, np.inf, np.inf, 1.5])     # 최대 상태 (속도는 1.5 이하)
    ocp.constraints.idxbx = np.array([3])  # 속도에 대한 상태 제약

    # 초기 상태 설정
    ocp.constraints.x0 = np.zeros(nx)

    # Solver 옵션 설정
    ocp.solver_options.qp_solver = "FULL_CONDENSING_QPOASES"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"

    return ocp

# 참조 궤적 설정 함수
def set_reference_trajectory(ocp_solver, ref_path, target_speed=1.0):
    """
    참조 궤적을 설정하는 함수
    ocp_solver: ACADOS OCP Solver 객체
    ref_path: 참조 경로, (N+1, 3) 크기의 numpy 배열 (x, y, yaw)
    target_speed: 목표 속도 (기본값: 1.0 m/s)
    """
    N = ocp_solver.dims.N
    ref_trajectory = np.zeros((N + 1, 4))  # [x, y, yaw, v]
    ref_trajectory[:, :3] = ref_path[:, :3]  # x, y, yaw 설정
    ref_trajectory[:, 3] = target_speed      # 속도 설정

    for i in range(N):
        ocp_solver.set(i, "yref", np.concatenate((ref_trajectory[i], [0.0, target_speed])))

    ocp_solver.set(N, "yref", ref_trajectory[-1])

# MPC 실행
def run_mpc():
    # OCP 생성
    ocp = create_ocp()

    # ACADOS Solver 생성
    ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
    sim_solver = AcadosSimSolver(ocp)

    # 초기 상태 설정
    x0 = np.array([0.0, 0.0, 0.0, 0.0])  # 초기 상태 [x, y, yaw, v]
    ocp_solver.set(0, "lbx", x0)
    ocp_solver.set(0, "ubx", x0)

    # 참조 궤적 설정 (예제)
    ref_path = np.zeros((ocp.dims.N + 1, 3))  # [x, y, yaw]
    ref_path[:, 0] = np.linspace(0, 10, ocp.dims.N + 1)  # x 참조
    ref_path[:, 1] = np.linspace(0, 5, ocp.dims.N + 1)   # y 참조
    ref_path[:, 2] = 0.0                                # yaw 참조
    set_reference_trajectory(ocp_solver, ref_path, target_speed=1.0)

    # MPC 실행
    for i in range(50):  # 50번 반복
        status = ocp_solver.solve()
        if status != 0:
            print(f"ACADOS solver failed at iteration {i} with status {status}")
            break

        # 최적화 결과 가져오기
        u0 = ocp_solver.get(0, "u")
        x0 = ocp_solver.get(1, "x")

        print(f"Iteration {i}: Control = {u0}, State = {x0}")

        # 시뮬레이션 업데이트
        sim_solver.set("x", x0)
        sim_solver.set("u", u0)
        sim_solver.solve()
        x0 = sim_solver.get("x")

        # 다음 단계 초기 상태 설정
        ocp_solver.set(0, "lbx", x0)
        ocp_solver.set(0, "ubx", x0)

if __name__ == "__main__":
    run_mpc()