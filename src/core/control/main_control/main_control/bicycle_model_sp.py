#! /usr/bin/env python3

# Bicycle Model Spline version

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan

def export_spline_bicycle_model() -> AcadosModel:

    model_name = "spline_bicycle_model"


    # constants
    WB = 1.566
    MASS = 500.0  # 차량 질량 [kg]
    ROLLING_RESISTANCE = 0.015  # 굴림 저항 계수
    AIR_DRAG = 0.3  # 공기 저항 계수

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

    # dynamics
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    yaw_dot = SX.sym("yaw_dot")
    v_dot = SX.sym("v_dot")
    s_dot = SX.sym("s_dot")

    xdot = vertcat(x_dot, y_dot, yaw_dot, v_dot, s_dot)

    # bicycle model equations with spline parameter and resistance forces
    # 저항력 계산
    rolling_resistance = ROLLING_RESISTANCE * MASS * 9.81  # 굴림 저항력
    air_resistance = AIR_DRAG * v**2  # 공기 저항력 (속도 제곱에 비례)
    total_resistance = rolling_resistance + air_resistance
    
    # 실제 가속도 = 입력 가속도 - 저항력/질량
    effective_a = a - total_resistance / MASS
    
    f_expl = vertcat(
        v * cos(yaw),         # x_dot
        v * sin(yaw),         # y_dot
        v / WB * tan(delta),  # yaw_dot
        effective_a,  # v_dot (저항력 고려)
        v  # s_dot (spline parameter changes with speed)
    )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = xdot
    model.u = controls
    model.name = model_name

    return model