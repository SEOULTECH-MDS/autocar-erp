#! /usr/bin/env python3

# Kinematic Bicycle Model

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan
import numpy as np

def export_bicycle_model() -> AcadosModel:

    model_name = "bicycle_model"

    # constants
    T = 3.0    # [s] prediction time
    DT = 0.1 # [s] time tick
    N = int(T / DT)  # number of steps
    WB = 1.566  # [m] wheelbase length

    # MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]  default: 45
    # MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    # MAX_SPEED = 1.5  # maximum speed [m/s]
    # MIN_SPEED = -1.5  # minimum speed [m/s]
    # MAX_ACCEL = 0.614  # maximum accel [m/ss] defualt: 1.0

    # states
    x = SX.sym("x")  # [m] x position
    y = SX.sym("y")  # [m] y position
    yaw = SX.sym("yaw")  # [rad] yaw angle
    v = SX.sym("v")  # [m/s] speed

    states = vertcat(x, y, yaw, v)

    # controls
    delta = SX.sym("delta")  # [rad] steering angle
    a = SX.sym("a")  # [m/s^2] acceleration

    controls = vertcat(delta, a)

    # dynamics
    x_dot = SX.sym("x_dot")  # [m/s] x velocity
    y_dot = SX.sym("y_dot")  # [m/s] y velocity
    yaw_dot = SX.sym("yaw_dot")  # [rad/s] yaw rate
    v_dot = SX.sym("v_dot")  # [m/s^2] acceleration

    xdot = vertcat(x_dot, y_dot, yaw_dot, v_dot)  # state derivative

    # bicycle model equations
    f_expl = vertcat(
        v * cos(yaw),         # x_dot
        v * sin(yaw),         # y_dot
        v / WB * tan(delta),  # yaw_dot
        a                     # v_dot
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
