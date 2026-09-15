#!/usr/bin/env python3
"""
plot_mpc_telemetry.py
Plots and compares simulation telemetry logs from PID and MPC runs:
- Lateral tracking error e_y(t)
- Speed profiles v(t)
- Acceleration & Steering inputs
- Solve time latency (MPC)
"""

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_logs(log_dir, prefix):
    state_file = os.path.join(log_dir, f"{prefix}_state.csv")
    control_file = os.path.join(log_dir, f"{prefix}_control.csv")

    # Fallback to standard names if prefix not found
    if not os.path.exists(state_file):
        state_file = os.path.join(log_dir, "state.csv")
    if not os.path.exists(control_file):
        control_file = os.path.join(log_dir, "control.csv")

    df_state = pd.read_csv(state_file) if os.path.exists(state_file) else None
    df_control = pd.read_csv(control_file) if os.path.exists(control_file) else None

    return df_state, df_control


def main():
    parser = argparse.ArgumentParser(description="Compare PID vs MPC PACSim telemetry logs.")
    parser.add_argument("--pid-dir", type=str, default="/workspace/PID_logs", help="Directory of PID logs")
    parser.add_argument("--mpc-dir", type=str, default="/workspace/MPC_logs", help="Directory of MPC logs")
    parser.add_argument("--output", type=str, default="/workspace/comparison_report.png", help="Output plot filename")
    args = parser.parse_args()

    pid_state, pid_ctrl = load_logs(args.pid_dir, "pid")
    mpc_state, mpc_ctrl = load_logs(args.mpc_dir, "mpc")

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("PACSim Controller Benchmark: PID Baseline vs NMPC", fontsize=16)

    # 1. Trajectory (X-Y)
    ax = axes[0, 0]
    if pid_state is not None:
        ax.plot(pid_state["x"], pid_state["y"], label="PID Baseline", color="gray", linestyle="--", alpha=0.8)
    if mpc_state is not None:
        ax.plot(mpc_state["x"], mpc_state["y"], label="NMPC (Frenet)", color="blue", linewidth=1.5)
    ax.set_title("Global Track Trajectory (Apex Cutting)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()

    # 2. Speed Profile v(t)
    ax = axes[0, 1]
    if pid_state is not None:
        ax.plot(pid_state["time"], pid_state["v"], label="PID Speed", color="gray", linestyle="--")
    if mpc_state is not None:
        ax.plot(mpc_state["time"], mpc_state["v"], label="NMPC Speed", color="green", linewidth=1.5)
    ax.set_title("Vehicle Speed Profile v(t)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.grid(True)
    ax.legend()

    # 3. Lateral Error e_y(t)
    ax = axes[1, 0]
    if pid_state is not None and "d" in pid_state:
        ax.plot(pid_state["time"], pid_state["d"], label="PID Lateral Error d", color="gray", linestyle="--")
    elif pid_state is not None and "e_y" in pid_state:
        ax.plot(pid_state["time"], pid_state["e_y"], label="PID Lateral Error e_y", color="gray", linestyle="--")
    if mpc_state is not None:
        ax.plot(mpc_state["time"], mpc_state["e_y"], label="NMPC Lateral Error e_y", color="crimson", linewidth=1.5)
    ax.axhline(1.5, color="black", linestyle=":", label="Track Boundary Limit (+1.5m)")
    ax.axhline(-1.5, color="black", linestyle=":", label="Track Boundary Limit (-1.5m)")
    ax.set_title("Lateral Deviation from Centerline")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("e_y [m]")
    ax.grid(True)
    ax.legend()

    # 4. Steering Command
    ax = axes[1, 1]
    if pid_ctrl is not None and "steer_wheel" in pid_ctrl:
        ax.plot(pid_ctrl["time"], pid_ctrl["steer_wheel"], label="PID Steering Wheel", color="gray", linestyle="--")
    if mpc_ctrl is not None and "steer_wheel_rad" in mpc_ctrl:
        ax.plot(mpc_ctrl["time"], mpc_ctrl["steer_wheel_rad"], label="NMPC Steering Wheel", color="purple", linewidth=1.5)
    ax.set_title("Steering Wheel Actuation")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Steering [rad]")
    ax.grid(True)
    ax.legend()

    # 5. Longitudinal Acceleration / Torque
    ax = axes[2, 0]
    if pid_ctrl is not None and "acc" in pid_ctrl:
        ax.plot(pid_ctrl["time"], pid_ctrl["acc"], label="PID Acc Command", color="gray", linestyle="--")
    if mpc_ctrl is not None and "a_cmd" in mpc_ctrl:
        ax.plot(mpc_ctrl["time"], mpc_ctrl["a_cmd"], label="NMPC Acc Command [m/s^2]", color="orange", linewidth=1.5)
    ax.set_title("Longitudinal Acceleration Command")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Accel [m/s^2]")
    ax.grid(True)
    ax.legend()

    # 6. acados Solver Solve Time
    ax = axes[2, 1]
    if mpc_ctrl is not None and "solve_time_us" in mpc_ctrl:
        solve_ms = mpc_ctrl["solve_time_us"] / 1000.0
        ax.plot(mpc_ctrl["time"], solve_ms, label="acados RTI Latency", color="teal")
        ax.axhline(10.0, color="red", linestyle="--", label="100Hz Deadline (10ms)")
        ax.set_title(f"acados RTI Solve Time (Mean: {solve_ms.mean():.2f} ms)")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Solve Time [ms]")
        ax.grid(True)
        ax.legend()

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f"✓ Telemetry benchmark report saved to: {args.output}")


if __name__ == "__main__":
    main()
