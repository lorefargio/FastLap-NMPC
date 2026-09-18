#!/usr/bin/env python3
"""
plot_mpc_timing.py
Plots per-iteration execution timing analysis for etdv_mpc from mpc_timing.csv:
1. Total loop time vs 100 Hz (10 ms) control budget deadline.
2. acados RTI solver latency: Linearization (prep) vs QP solve (feedback).
3. Sub-millisecond stage breakdown: projection, horizon updates, actuator publishing.
4. QP iterations and latency distribution histogram.
"""

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Plot per-iteration MPC execution timings.")
    parser.add_argument("--timing-file", type=str, default="MPC_logs/mpc_timing.csv",
                        help="Path to mpc_timing.csv")
    parser.add_argument("--output", type=str, default="MPC_logs/timing_report.png",
                        help="Output image path")
    args = parser.parse_args()

    if not os.path.exists(args.timing_file):
        print(f"Error: {args.timing_file} not found.")
        return

    df = pd.read_csv(args.timing_file)
    if df.empty:
        print("Timing file is empty.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"ETDV NMPC Execution Latency Benchmark ({len(df)} Iterations)", fontsize=16)

    # 1. Total Loop Time vs Control Budget
    ax = axes[0, 0]
    ax.plot(df["time_sec"], df["total_loop_ms"], label="Total Loop Iteration", color="royalblue", alpha=0.85)
    ax.plot(df["time_sec"], df["solver_ms"], label="acados RTI Solver", color="crimson", linewidth=1.2)
    ax.axhline(10.0, color="red", linestyle="--", linewidth=1.5, label="100 Hz Budget (10.0 ms)")
    ax.axhline(df["total_loop_ms"].mean(), color="navy", linestyle=":", label=f"Mean Loop: {df['total_loop_ms'].mean():.2f} ms")
    ax.set_title(f"Per-Iteration Latency (Max: {df['total_loop_ms'].max():.2f} ms, P99: {np.percentile(df['total_loop_ms'], 99):.2f} ms)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Latency [ms]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # 2. acados RTI Breakdown: Preparation (Linearization) vs Feedback (QP)
    ax = axes[0, 1]
    if "lin_time_ms" in df and "qp_time_ms" in df:
        ax.plot(df["time_sec"], df["lin_time_ms"], label=f"Linearization / Prep (Mean: {df['lin_time_ms'].mean():.2f} ms)", color="darkorange")
        ax.plot(df["time_sec"], df["qp_time_ms"], label=f"QP Solve / Feedback (Mean: {df['qp_time_ms'].mean():.2f} ms)", color="forestgreen")
    else:
        ax.plot(df["time_sec"], df["solver_ms"], label="Total Solver", color="crimson")
    ax.set_title(f"acados RTI Phases (Solver Mean: {df['solver_ms'].mean():.2f} ms)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Latency [ms]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # 3. Pipeline Sub-components Latency in Microseconds
    ax = axes[1, 0]
    if "proj_us" in df:
        ax.plot(df["time_sec"], df["proj_us"], label=f"Frenet Projection (Mean: {df['proj_us'].mean():.1f} us)", color="purple")
    if "horizon_us" in df:
        ax.plot(df["time_sec"], df["horizon_us"], label=f"SpeedGovernor & Horizon (Mean: {df['horizon_us'].mean():.1f} us)", color="teal")
    if "publish_us" in df:
        ax.plot(df["time_sec"], df["publish_us"], label=f"Torque Map & Publish (Mean: {df['publish_us'].mean():.1f} us)", color="olive")
    ax.set_title("C++ Node Pipeline Stages Latency [microseconds]")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Time [us]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # 4. Latency Distribution Histogram
    ax = axes[1, 1]
    ax.hist(df["total_loop_ms"], bins=40, color="steelblue", edgecolor="black", alpha=0.7, label="Total Loop")
    ax.hist(df["solver_ms"], bins=40, color="crimson", edgecolor="black", alpha=0.7, label="Solver")
    ax.axvline(10.0, color="red", linestyle="--", linewidth=1.5, label="100 Hz Limit (10 ms)")
    ax.set_title("Iteration Latency Distribution")
    ax.set_xlabel("Latency [ms]")
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    plt.savefig(args.output, dpi=150)
    print(f"✓ Saved timing analysis plot to: {args.output}")

    # Print summary statistics
    print("\n=== ETDV NMPC Timing Summary ===")
    print(f"Total Iterations: {len(df)}")
    print(f"Control Rate:     100 Hz (10.0 ms budget)")
    print(f"Loop Latency:     Mean = {df['total_loop_ms'].mean():.3f} ms | Min = {df['total_loop_ms'].min():.3f} ms | Max = {df['total_loop_ms'].max():.3f} ms | P99 = {np.percentile(df['total_loop_ms'], 99):.3f} ms")
    print(f"Solver Latency:   Mean = {df['solver_ms'].mean():.3f} ms | Min = {df['solver_ms'].min():.3f} ms | Max = {df['solver_ms'].max():.3f} ms")
    if "lin_time_ms" in df and "qp_time_ms" in df:
        print(f"  Linearization:  Mean = {df['lin_time_ms'].mean():.3f} ms")
        print(f"  QP Solve:       Mean = {df['qp_time_ms'].mean():.3f} ms")
    overruns = (df['total_loop_ms'] > 10.0).sum()
    print(f"Overruns (>10ms): {overruns} ({overruns/len(df)*100.0:.2f}%)")
    print("================================")


if __name__ == "__main__":
    main()
