import os
import time
import numpy as np
from acados_template import AcadosOcpSolver
from generate_c_code import create_ocp


def test_solver():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    c_code_dir = os.path.join(script_dir, "..", "c_generated_code")
    json_path = os.path.join(script_dir, "acados_ocp.json")

    print("=== Loading acados OCP Solver ===")
    try:
        ocp = create_ocp()
        ocp.code_export_directory = c_code_dir
        solver = AcadosOcpSolver(ocp, json_file=json_path)
    except Exception as e:
        print(f"Error loading solver: {e}")
        print("Please run python3 generate_c_code.py first.")
        return

    # Set realistic initial condition
    # x = [s, e_y, e_psi, v, delta]
    x0 = np.array([0.0, 0.3, 0.05, 8.0, 0.0])
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)

    # Set mock curve parameters along horizon
    N = 30
    for k in range(N):
        # A gentle left curve: kappa = 0.05 (R = 20m)
        p_k = np.array([0.05, 1.5, 1.5, 1.0])
        solver.set(k, "p", p_k)

    print("\n--- Running 10 Real-Time Iterations (RTI) ---")
    timings = []
    for i in range(10):
        t_start = time.perf_counter()
        status = solver.solve()
        t_elapsed = (time.perf_counter() - t_start) * 1000.0  # ms
        timings.append(t_elapsed)

        u0 = solver.get(0, "u")
        x1 = solver.get(1, "x")
        print(f"Iter {i:02d}: Status={status} | Solve Time = {t_elapsed:.3f} ms | u0 = [acc: {u0[0]:.2f} m/s^2, v_delta: {u0[1]:.2f} rad/s]")

    avg_time = np.mean(timings)
    print(f"\n✓ Mean Solve Time: {avg_time:.3f} ms (Real-Time Budget at 100Hz = 10.0 ms)")
    if avg_time < 2.0:
        print("✓ EXCELLENT: Solver is easily fast enough for 100 Hz real-time operation!")
    else:
        print("! Warning: Solver time exceeds 2.0 ms")


if __name__ == "__main__":
    test_solver()
