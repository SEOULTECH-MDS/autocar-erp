import cvxpy
import math
import numpy as np

from autocar_nav import normalise_angle


NX = 4  # x = x, y, v, yaw
NU = 2  # u = [v, steer]
T = 5  # horizon length 5s: default

# mpc parameters
R = np.diag([0.01, 0.05])  # input cost matrix default: 0.01, 0.01
Rd = np.diag([0.01, 1.0])  # input difference cost matrix default: 0.01, 1.0
Q = np.diag([1.2, 1.2, 0.5, 0.5])  # state cost matrix default: 1.0, 1.0, 0.5, 0.5
Qf = Q  # state final matrix

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 1.5  # [m/s] target speed defualt: 10.0 / 3.6
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

WB = 1.566  # [m] default : 2.5, ERP42 : 1.566

MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]  default: 45
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 1.5  # maximum speed [m/s]
MIN_SPEED = -1.5  # minimum speed [m/s]
MAX_ACCEL = 0.614  # maximum accel [m/ss] defualt: 1.0

class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    return normalise_angle(angle)


def get_linear_model_matrix(v, phi, delta):
    """
    선형 모델 행렬 계산
    v: 속도
    phi: yaw
    delta: 조향각
    """

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[0, 0] = DT * math.cos(phi)
    B[1, 0] = DT * math.sin(phi)
    B[2, 0] = 1.0
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def update_state(state, v, delta):
    """
    차량 상태 업데이트
    state: 현재 상태
    v: 속도
    delta: 조향각
    """

    # 입력 값 체크
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + v * math.cos(state.yaw) * DT
    state.y = state.y + v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + v / WB * math.tan(delta) * DT
    state.v = v

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    """
    행렬을 numpy 배열로 변환
    """
    return np.array(x).flatten()

def calc_nearest_index(state, cx, cy, cyaw, pind):
    """
    가장 가까운 인덱스 계산
    state: 차량의 현재 상태
    cx, cy, cyaw : 경로의 x, y, yaw 값
    pind: 이전 인덱스
    """

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind # 가장 가까운 인덱스, 인덱스까지 거리 반환


def predict_motion(x0, ov, od, xref):
    """
    상태 예측
    x0: 초기상태
    ov: 속도 입력
    od: 조향각 입력
    xref: 참조 궤적 행렬 (예측시간 T 동안 목표궤적)
    """
    xbar = xref * 0.0 # xref와 같은 크기의 0행렬 생성, xbar는 예측된 상태값을 저장

    # 초기 상태값 저장
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    # state_list = [x0.x, x0.y, x0.v, x0.yaw]
    # for i, value in enumerate(state_list):
    #     xbar[i, 0] = value

    # 상태 예측
    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (vi, di, i) in zip(ov, od, range(1, T + 1)): # T만큼 반복
        state = update_state(state, vi, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, ov, od):
    """
    반복적 선형 MPC 제어
    xref: 목표 궤적
    x0: 초기 상태
    dref: 목표 조향각
    ov: 초기 속도 입력
    od: 초기 조향각 입력
    """
    ox, oy, oyaw, ov_state = None, None, None, None

    if ov is None or od is None:
        ov = [TARGET_SPEED] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, ov, od, xref)
        pov, pod = ov[:], od[:] # 이전 속도, 조향각 저장
        ov, od, ox, oy, oyaw, ov_state = linear_mpc_control(xref, xbar, x0, dref) # 최적화 문제를 풀어 새로운 속도, 조향각 계산
        du = sum(abs(ov - pov)) + sum(abs(od - pod))  # 입력 변화량 계산
        if du <= DU_TH: # du가 충분히 작아지면 수렴한 것으로 판단
            break
    else:
        print("Iterative is max iter")

    return ov, od, ox, oy, oyaw, ov_state  # 최적화한 속도, 조향각, 상태값 반환


def linear_mpc_control(xref, xbar, x0, dref):
    """
    선형 MPC 최적화
    xref: 목표 궤적
    xbar: 예측 궤적
    x0: 현재 차량 상태
    dref: 목표 조향각
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0 # 비용함수
    constraints = [] # 제약조건

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R) # 제어 입력을 최소화 하는 비용함수

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q) # 목표 궤적과 현재 상태의 차이를 최소화 하는 비용함수

        # 시스템 모델 제약
        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        # 조향각의 급격한 변화 방지
        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf) # 최종 상태와 목표 상태의 차이를 최소화 하는 비용함수

    # 차량의 상태, 속도, 가속도, 조향각 제한
    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_SPEED]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    # 최적화 문제 정의, 해결
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    # 최적화 값이 존재하면 결과 반환
    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov_state = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        ov = get_nparray_from_matrix(u.value[0, :])
        od = get_nparray_from_matrix(u.value[1, :])
    # 최적화 값이 존재하지 않으면 None 반환
    else:
        print("Error: Cannot solve mpc..")
        ov, od, ox, oy, oyaw, ov_state = None, None, None, None, None, None

    return ov, od, ox, oy, oyaw, ov_state


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    """
    참조 궤적 계산
    state: 차량의 현재 상태
    cx, cy, cyaw: local path의 x, y, yaw 값
    ck: 곡률
    sp: 속도 프로파일
    dl: 경로 간격
    pind: 이전 인덱스
    """
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind
    # 차량이 가장 가까운 경로 인덱스를 따르도록 초기 상태 설정
    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # 조향각 초기값

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref