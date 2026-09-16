#!/usr/bin/env python3
"""
generate_c_code.py
Configures the Optimal Control Problem (OCP) for the Kinematic Frenet Model
and compiles high-performance C code using acados_template.

Single Source of Truth (SSOT):
Loads configuration directly from etdv_mpc/config/mpc_params.yaml to ensure
complete consistency across model constraints, reference speeds, and solver bounds.
"""

import os
import sys
import ctypes
import numpy as np
import casadi as ca

# Ensure acados environment variables and dynamic linker paths are configured
if "ACADOS_SOURCE_DIR" not in os.environ:
    os.environ["ACADOS_SOURCE_DIR"] = "/opt/acados"

acados_lib_dir = os.path.join(os.environ["ACADOS_SOURCE_DIR"], "lib")
ld_path = os.environ.get("LD_LIBRARY_PATH", "")
if acados_lib_dir not in ld_path:
    os.environ["LD_LIBRARY_PATH"] = f"{acados_lib_dir}:{ld_path}"

# Preload acados shared libraries to resolve any circular dependencies for ctypes
for lib_name in ["libblasfeo.so", "libhpipm.so", "libqpOASES_e.so", "libacados.so"]:
    lib_path = os.path.join(acados_lib_dir, lib_name)
    if os.path.exists(lib_path):
        try:
            ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
        except Exception:
            pass

from acados_template import AcadosOcp, AcadosOcpSolver
from kinematic_frenet_model import export_kinematic_frenet_model


def load_yaml_params() -> dict:
    """Loads configuration parameters from mpc_params.yaml with safe fallbacks."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(script_dir, "..", "config", "mpc_params.yaml")
    params = {
        "mpc_dt": 0.05,
        "default_track_width": 3.0,
        "track_margin": 0.90,
        "effective_mu": 1.0,
        "max_accel": 3.5,
        "min_accel": -8.0,
        "max_straight_speed": 22.5,
        "speed_scale": 0.90,
    }

    if os.path.exists(yaml_path):
        try:
            import yaml
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
                if data and "mpc_controller" in data and "ros__parameters" in data["mpc_controller"]:
                    params.update(data["mpc_controller"]["ros__parameters"])
                    print(f"[SSOT] Loaded parameters from {yaml_path}")
        except Exception as e:
            # Fallback simple line-based parser if yaml library is unavailable
            try:
                with open(yaml_path, "r") as f:
                    for line in f:
                        stripped = line.strip()
                        if ":" in stripped and not stripped.startswith("#"):
                            k, v = stripped.split(":", 1)
                            k = k.strip()
                            v = v.strip().split("#")[0].strip()
                            if k in params:
                                try:
                                    params[k] = float(v)
                                except ValueError:
                                    pass
                print(f"[SSOT] Loaded parameters from {yaml_path} (native fallback parser)")
            except Exception as e2:
                print(f"[SSOT] Warning: Could not read {yaml_path}: {e2}. Using embedded defaults.")
    else:
        print(f"[SSOT] {yaml_path} not found. Using embedded defaults.")

    return params


def create_ocp() -> AcadosOcp:
    params = load_yaml_params()
    ocp = AcadosOcp()
    model = export_kinematic_frenet_model()
    ocp.model = model
    ocp.name = model.name

    # =========================================================================
    # Horizon Discretization (from mpc_params.yaml)
    # =========================================================================
    N = 30
    dt = float(params.get("mpc_dt", 0.05))
    Tf = N * dt

    # Compatibility with modern acados (v0.5.4+)
    if hasattr(ocp, "code_gen_options") and hasattr(ocp.code_gen_options, "N_horizon"):
        ocp.code_gen_options.N_horizon = N
    else:
        ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # =========================================================================
    # Initial Parameters (p = [kappa, w_l, w_r, mu])
    # =========================================================================
    default_w = float(params.get("default_track_width", 3.0)) / 2.0
    effective_mu = float(params.get("effective_mu", 1.0))

    kappa_init = 0.0
    w_l_init = default_w
    w_r_init = default_w
    mu_init = effective_mu
    p_init = np.array([kappa_init, w_l_init, w_r_init, mu_init])

    ocp.parameter_values = p_init

    # =========================================================================
    # Cost Formulation: Maximum Progress & Lap Time Minimization
    # =========================================================================
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # Stage cost residual y = [v, e_y, e_psi, delta, a, v_delta]
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

    # Cost weights (Tuned for smooth tracking without twitching or weaving)
    w_v       = 2.5      # Target speed tracking weight
    q_ey      = 15.0     # Damped lateral tracking penalty
    q_epsi    = 18.0     # Heading tangent alignment penalty
    r_delta   = 0.30     # Steering angle regularization
    r_a       = 0.40     # Longitudinal acceleration smoothness
    r_vdelta  = 3.50     # High steering rate penalty (forces smooth fluid steering)

    W = np.diag([w_v, q_ey, q_epsi, r_delta, r_a, r_vdelta])
    W_e = np.diag([w_v * 1.5, q_ey * 1.5, q_epsi * 1.5, r_delta])

    ocp.cost.W = W
    ocp.cost.W_e = W_e

    # Reference values: v_target directly driven by max_straight_speed
    max_straight_speed = float(params.get("max_straight_speed", 22.5))
    v_target = max_straight_speed
    ocp.cost.yref = np.array([v_target, 0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.yref_e = np.array([v_target, 0.0, 0.0, 0.0])

    # =========================================================================
    # Constraints & Bounds
    # =========================================================================
    # Track margin from YAML (car half-width + safety margin)
    car_margin = float(params.get("track_margin", 0.90))
    e_y_min = - (w_r_init - car_margin)
    e_y_max =   (w_l_init - car_margin)

    # Upper bound on velocity: generous headroom (at least 35 m/s or 1.3 * v_max)
    # Prevents solver infeasibility when max_straight_speed is increased!
    v_ub = max(35.0, max_straight_speed * 1.3)

    ocp.constraints.idxbx = np.array([1, 3, 4])
    ocp.constraints.lbx = np.array([e_y_min,  0.0, -0.52])
    ocp.constraints.ubx = np.array([e_y_max, v_ub,  0.52])

    # Soft constraints on:
    # 1. Lateral deviation e_y (idxsbx = 0)
    # 2. Non-linear friction circle con_h (idxsh = 0)
    # Softening friction circle guarantees QP feasibility at high speeds and corner entries!
    ocp.constraints.idxsbx = np.array([0])
    ocp.constraints.idxsh = np.array([0])

    # Penalties for slack variables: [e_y_slack, friction_circle_slack]
    ocp.cost.zl = np.array([200.0, 100.0]) # L1 linear penalty
    ocp.cost.zu = np.array([200.0, 100.0])
    ocp.cost.Zl = np.array([1000.0, 500.0]) # L2 quadratic penalty
    ocp.cost.Zu = np.array([1000.0, 500.0])

    # Input box constraints: [a, v_delta]
    max_accel = float(params.get("max_accel", 3.5))
    min_accel = float(params.get("min_accel", -8.0))

    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([min_accel, -1.5]) # Steering rate limit: 1.5 rad/s
    ocp.constraints.ubu = np.array([max_accel,  1.5])

    # Non-linear constraint: Friction Circle (normalized: <= 1.0)
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

    # Modern acados code export options
    if hasattr(ocp, "code_gen_options") and hasattr(ocp.code_gen_options, "code_export_directory"):
        ocp.code_gen_options.code_export_directory = c_code_dir
    else:
        ocp.code_export_directory = c_code_dir

    json_path = os.path.join(c_code_dir, "acados_ocp.json")
    if hasattr(ocp, "code_gen_options") and hasattr(ocp.code_gen_options, "json_file"):
        ocp.code_gen_options.json_file = json_path

    # Generate C code
    try:
        solver = AcadosOcpSolver(ocp, json_file=json_path)
        print(f"✓ acados C-code generated and compiled successfully in: {c_code_dir}")
        return solver
    except OSError as e:
        # On some systems ctypes fails to load libqpOASES_e.so inside Python, but
        # the C source files were already generated and compiled for CMake!
        print(f"[Info] acados C-code generated in: {c_code_dir}")
        print(f"[Info] Python loader note: {e}")
        print(f"✓ C source code is ready for CMake compilation via 'colcon build'.")
        return None


if __name__ == "__main__":
    generate_code()
