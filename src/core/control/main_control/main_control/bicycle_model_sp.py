#! /usr/bin/env python3

# Bicycle Model Spline version

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan, fabs

def export_spline_bicycle_model() -> AcadosModel:

    model_name = "spline_bicycle_model"


    # constants
    # WB = 1.566
    WB = 1.000  # 휠베이스 [m]
    MASS = 1000.0  # 차량 질량 [kg]
    ROLLING_RESISTANCE = 0.015  # 굴림 저항 계수

    # states
    x = SX.sym("x")
    y = SX.sym("y")
    yaw = SX.sym("yaw")
    v = SX.sym("v")
    s = SX.sym("s")  # spline parameter

    states = vertcat(x, y, yaw, v, s)

    # controls
    delta = SX.sym("delta") # steering angle
    a = SX.sym("a") # acceleration

    controls = vertcat(delta, a)

    # controls (속도 지연 모델용)
    # delta = SX.sym("delta") # steering angle
    # v_cmd = SX.sym("v_cmd") # commanded velocity
    # tau = 0.5
    # controls = vertcat(delta, v_cmd)

    # dynamics
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    yaw_dot = SX.sym("yaw_dot")
    v_dot = SX.sym("v_dot")
    s_dot = SX.sym("s_dot")

    xdot = vertcat(x_dot, y_dot, yaw_dot, v_dot, s_dot)

    # 저항력 계산
    rolling_resistance = ROLLING_RESISTANCE * MASS * 9.81  # 굴림 저항력
    
    # 실제 가속도 = 입력 가속도 - 저항력/질량
    # effective_a = a - rolling_resistance / MASS
    effective_a = a

    # simplified bicycle model (가속도 입력 모델)
    f_expl = vertcat(
        v * cos(yaw),         # x_dot
        v * sin(yaw),         # y_dot
        v / WB * tan(delta),  # yaw_dot
        effective_a,  # v_dot (저항력 고려)
        fabs(v)  # s_dot (spline parameter changes with speed)
    )

    # simplified bicycle model (속도 지연 모델 )
    '''
    이 모델을 사용할 경우 제어 입력은 delta, v_cmd가 되고, mpc loop에서 속도 입력을 u_opt[0, 1]로 주어야 함.
    '''
    # f_expl = vertcat(
    #     v * cos(yaw),         # x_dot
    #     v * sin(yaw),         # y_dot
    #     v / WB * tan(delta),  # yaw_dot
    #     (v_cmd - v) / tau,  # v_dot 
    #     v  # s_dot (spline parameter changes with speed)
    # )


    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = xdot
    model.u = controls
    model.name = model_name

    return model