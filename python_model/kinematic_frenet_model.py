"""
kinematic_frenet_model.py
Defines the symbolic Kinematic Single-Track (Bicycle) Model in Frenet coordinates
using CasADi for acados OCP code generation.
"""

import casadi as ca
from acados_template import AcadosModel


def export_kinematic_frenet_model() -> AcadosModel:
    model_name = "kinematic_frenet"

    # =========================================================================
    # Model Parameters & Vehicle Constants (from pacsim vehicleModel.yaml)
    # =========================================================================
    lf = 0.69   # Distance from CG to front axle [m]
    lr = 0.84   # Distance from CG to rear axle [m]
    L = lf + lr # Wheelbase [m] = 1.53 m
    g = 9.81    # Gravity acceleration [m/s^2]

    # =========================================================================
    # State Variables (x in R^5)
    # =========================================================================
    s     = ca.SX.sym("s")      # Arc length along centerline [m]
    e_y   = ca.SX.sym("e_y")    # Lateral deviation from centerline [m] (+ left, - right)
    e_psi = ca.SX.sym("e_psi")  # Heading error w.r.t. track tangent [rad]
    v     = ca.SX.sym("v")      # Vehicle speed at CG [m/s]
    delta = ca.SX.sym("delta")  # Wheel steering angle [rad]

    x = ca.vertcat(s, e_y, e_psi, v, delta)

    # State derivatives
    s_dot     = ca.SX.sym("s_dot")
    e_y_dot   = ca.SX.sym("e_y_dot")
    e_psi_dot = ca.SX.sym("e_psi_dot")
    v_dot     = ca.SX.sym("v_dot")
    delta_dot = ca.SX.sym("delta_dot")

    xdot = ca.vertcat(s_dot, e_y_dot, e_psi_dot, v_dot, delta_dot)

    # =========================================================================
    # Control Inputs (u in R^2)
    # =========================================================================
    a       = ca.SX.sym("a")        # Net longitudinal acceleration [m/s^2]
    v_delta = ca.SX.sym("v_delta")  # Steering angular velocity [rad/s]

    u = ca.vertcat(a, v_delta)

    # =========================================================================
    # Online Dynamic Parameters (p in R^4)
    # =========================================================================
    kappa = ca.SX.sym("kappa")  # Reference curvature along horizon [1/m]
    w_l   = ca.SX.sym("w_l")    # Left lane width limit [m]
    w_r   = ca.SX.sym("w_r")    # Right lane width limit [m]
    mu    = ca.SX.sym("mu")     # Effective friction coefficient [-]

    p = ca.vertcat(kappa, w_l, w_r, mu)

    # =========================================================================
    # Kinematics in Frenet Frame
    # =========================================================================
    # Kinematic side-slip angle at vehicle CG
    beta = ca.atan((lr / L) * ca.tan(delta))

    # Path curvature scaling denominator (1 - e_y * kappa)
    # Safe protection against numerical singularity if vehicle leaves track envelope
    denom = ca.fmax(0.05, 1.0 - e_y * kappa)

    # Global yaw rate of vehicle
    psi_dot_veh = (v * ca.cos(beta) / L) * ca.tan(delta)

    # Progress along centerline
    s_dot_expr = (v * ca.cos(e_psi + beta)) / denom

    # Lateral deviation rate
    e_y_dot_expr = v * ca.sin(e_psi + beta)

    # Heading error rate (d(e_psi)/dt = psi_dot_veh - kappa * s_dot)
    e_psi_dot_expr = psi_dot_veh - kappa * s_dot_expr

    # Acceleration and steering rate integration
    v_dot_expr = a
    delta_dot_expr = v_delta

    # Explicit ODE rhs
    f_expl = ca.vertcat(
        s_dot_expr,
        e_y_dot_expr,
        e_psi_dot_expr,
        v_dot_expr,
        delta_dot_expr
    )

    # Implicit ODE (f_impl = 0)
    f_impl = xdot - f_expl

    # =========================================================================
    # Non-linear Path Constraints (Friction Circle & Lateral Accel)
    # =========================================================================
    # Lateral acceleration at CG: a_lat = v * psi_dot_veh
    a_lat = v * psi_dot_veh
    # Combined tire force constraint: a_lon^2 + a_lat^2 <= (mu * g)^2
    # Normalized friction circle: (a / (mu * g))^2 + (a_lat / (mu * g))^2 <= 1.0
    friction_circle = (a**2 + a_lat**2) / ((mu * g)**2)

    # Store in acados model
    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    # Expose auxiliary expressions
    model.con_h_expr = friction_circle
    model.s_dot = s_dot_expr

    return model
