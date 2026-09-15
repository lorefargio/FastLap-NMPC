#!/usr/bin/env python3
"""
generate_c_code.py
Configures the Optimal Control Problem (OCP) for the Kinematic Frenet Model
and compiles high-performance C code using acados_template.
"""

import os
import shutil
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver
from kinematic_frenet_model import export_kinematic_frenet_model


def create_ocp() -> AcadosOcp:
    ocp = AcadosOcp()
    model = export_kinematic_frenet_model()
    ocp.model = model
    ocp.name = model.name

    # =========================================================================
    # Horizon Discretization
    # =========================================================================
    N = 30              # Number of shooting nodes
    dt = 0.05           # Discretization step [s] (Lookahead = N * dt = 1.5 s)
    Tf = N * dt         # Total prediction time [s]

    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # =========================================================================
    # Initial Parameters (p = [kappa, w_l, w_r, mu])
    # =========================================================================
    kappa_init = 0.0
    w_l_init = 1.5      # Left lane width from centerline [m]
    w_r_init = 1.5      # Right lane width from centerline [m]
    mu_init = 1.0       # Effective friction coefficient
    p_init = np.array([kappa_init, w_l_init, w_r_init, mu_init])

    ocp.parameter_values = p_init

    # =========================================================================
    # Cost Formulation: Maximum Progress & Lap Time Minimization
    # =========================================================================
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # Stage cost residual y = [v, e_y, e_psi, delta, a, v_delta]
    # Driving v towards v_max maximizes progress s_dot and minimizes lap time.
    ocp.model.cost_y_expr = ca.vertcat(
        model.x[3],      # v
        model.x[1],      # e_y
        model.x[2],      # e_psi
        model.x[4],      # delta
        model.u[0],      # a
        model.u[1]       # v_delta
    )

    # Terminal cost residual y_e = [v, e_y, e_psi, delta]
    ocp.model.cost_y_expr_e = ca.vertcat(
        model.x[3],      # v
        model.x[1],      # e_y
        model.x[2],      # e_psi
        model.x[4]       # delta
    )

    # Cost weights
    w_v       = 20.0     # High weight on maximizing velocity (lap time minimization)
    q_ey      = 0.3      # Low weight on lateral deviation (allows apex cutting & racing line)
    q_epsi    = 1.5      # Alignment with track direction
    r_delta   = 0.05     # Steering angle regularization
    r_a       = 0.05     # Acceleration smoothness
    r_vdelta  = 0.2      # Steering angular rate penalty (prevents chattering)

    W = np.diag([w_v, q_ey, q_epsi, r_delta, r_a, r_vdelta])
    W_e = np.diag([w_v * 1.5, q_ey * 1.5, q_epsi * 1.5, r_delta])

    ocp.cost.W = W
    ocp.cost.W_e = W_e

    # Reference values (target maximum speed)
    v_target = 22.0  # m/s (~80 km/h)
    ocp.cost.yref = np.array([v_target, 0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.yref_e = np.array([v_target, 0.0, 0.0, 0.0])

    # =========================================================================
    # Constraints
    # =========================================================================
    # State box constraints: [e_y, v, delta]
    # States index: s=0, e_y=1, e_psi=2, v=3, delta=4
    # Track margin: car half-width (0.7m) + safety margin (0.15m) = 0.85m
    car_margin = 0.85
    e_y_min = - (w_r_init - car_margin)
    e_y_max =   (w_l_init - car_margin)

    ocp.constraints.idxbx = np.array([1, 3, 4])
    ocp.constraints.lbx = np.array([e_y_min,  0.0, -0.52])
    ocp.constraints.ubx = np.array([e_y_max, 25.0,  0.52])

    # Soft constraints on lateral error (e_y) to guarantee QP feasibility
    ocp.constraints.idxsbx = np.array([0])  # Soften e_y (first element of idxbx)
    ocp.cost.zl = np.array([100.0])         # L1 penalty
    ocp.cost.zu = np.array([100.0])
    ocp.cost.Zl = np.array([500.0])         # L2 penalty
    ocp.cost.Zu = np.array([500.0])

    # Input box constraints: [a, v_delta]
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([-8.0, -1.5])   # Max braking: -8 m/s^2, steer rate: -1.5 rad/s
    ocp.constraints.ubu = np.array([ 3.5,  1.5])   # Max accel: 3.5 m/s^2, steer rate: 1.5 rad/s

    # Non-linear constraint: Friction Circle
    # (a / (mu * g))^2 + (a_lat / (mu * g))^2 <= 1.0
    ocp.constraints.lh = np.array([0.0])
    ocp.constraints.uh = np.array([1.0])

    # Initial state constraint (x0)
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    ocp.constraints.x0 = x0

    # =========================================================================
    # Solver Options: Real-Time Iteration (RTI)
    # =========================================================================
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = 5
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # Real-Time Iteration scheme
    ocp.solver_options.print_level = 0

    return ocp


def generate_code():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    c_code_dir = os.path.join(script_dir, "..", "c_generated_code")
    
    print(f"=== Starting acados C-code generation ===")
    print(f"Target code directory: {c_code_dir}")

    os.makedirs(c_code_dir, exist_ok=True)
    ocp = create_ocp()
    ocp.code_export_directory = c_code_dir
    json_path = os.path.join(script_dir, "acados_ocp.json")
    
    # Generate C code and compile shared library
    solver = AcadosOcpSolver(ocp, json_file=json_path)
    print(f"✓ acados C-code generated successfully in: {c_code_dir}")
    return solver


if __name__ == "__main__":
    generate_code()
