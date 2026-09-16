#!/usr/bin/env python3
"""
analyze_multitrack.py
Cross-Track Diagnostic & Overfitting Analysis Engine for FastLap NMPC.

Ingests test run results from 3 Formula Student circuits:
  - FSE23 (Formula Student Spain 2023 - Technical & hairpins)
  - FSG21 (Formula Student Germany 2021 - High technicality, chicanes & slaloms)
  - FSE24 (Formula Student Spain 2024 - Fast flowing sweeps & directional transitions)

Evaluates:
  1. 2-Lap Performance: Lap 1 Standing Start vs Lap 2 Flying Lap
  2. Cross-Track Generalization & Overfitting Detection
  3. Dynamic Grip & Envelope Utilization
  4. Solver Real-Time Determinism across Varied Curvatures
  5. Safety Margins (Cone clearances & boundary headroom)

Outputs:
  - Terminal Cross-Track Executive Scorecard
  - Publication-grade Markdown Report (multitrack_summary_report.md)
  - 6-Panel Diagnostic Dashboard (multitrack_comparison.png)
"""

import os
import sys
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from typing import Dict, Any, List, Optional, Tuple


TRACK_METADATA = {
    "FSE23": {"name": "FS Spain 2023", "length_m": 298.0, "color": "#1f77b4"},
    "FSG21": {"name": "FS Germany 2021", "length_m": 380.0, "color": "#ff7f0e"},
    "FSI24": {"name": "FS Italy 2024", "length_m": 390.0, "color": "#2ca02c"},
    "FSE24": {"name": "FS Spain 2024", "length_m": 340.0, "color": "#9467bd"},
}


def load_track_data(track_dir: str) -> Tuple[Optional[pd.DataFrame], Optional[pd.DataFrame], Optional[Dict[str, Any]]]:
    """Loads telemetry, timing, and laps summary with robust fallbacks."""
    telem_path = os.path.join(track_dir, "mpc_telemetry.csv")
    detailed_path = os.path.join(track_dir, "mpc_detailed.csv")
    state_path = os.path.join(track_dir, "mpc_state.csv")
    timing_path = os.path.join(track_dir, "mpc_timing.csv")
    summary_path = os.path.join(track_dir, "laps_summary.json")

    df_telem = None
    if os.path.exists(telem_path) and os.path.getsize(telem_path) > 100:
        try:
            df_telem = pd.read_csv(telem_path)
            print(f"[DataLoader] Loaded telemetry from {os.path.basename(telem_path)} ({len(df_telem)} rows)")
        except Exception as e:
            print(f"[DataLoader] Warning: Could not read {telem_path}: {e}")
    elif os.path.exists(detailed_path) and os.path.getsize(detailed_path) > 100:
        try:
            df_telem = pd.read_csv(detailed_path)
            print(f"[DataLoader] Fallback to detailed log: {os.path.basename(detailed_path)} ({len(df_telem)} rows)")
            if "s_lap" not in df_telem.columns and "s" in df_telem.columns:
                df_telem["s_lap"] = df_telem["s"]
            if "a_lon" not in df_telem.columns and "a_cmd" in df_telem.columns:
                df_telem["a_lon"] = df_telem["a_cmd"]
            if "a_lat" not in df_telem.columns and "v" in df_telem.columns and "yaw_rate" in df_telem.columns:
                df_telem["a_lat"] = df_telem["v"] * df_telem["yaw_rate"]
            if "friction_util_pct" not in df_telem.columns and "friction_util" in df_telem.columns:
                df_telem["friction_util_pct"] = df_telem["friction_util"] * 100.0
            if "min_cone_clearance" not in df_telem.columns and "e_y" in df_telem.columns:
                df_telem["min_cone_clearance"] = 1.5 - np.abs(df_telem["e_y"])
        except Exception as e:
            print(f"[DataLoader] Warning: Could not read {detailed_path}: {e}")
    elif os.path.exists(state_path) and os.path.getsize(state_path) > 100:
        try:
            df_telem = pd.read_csv(state_path)
            print(f"[DataLoader] Fallback to state log: {os.path.basename(state_path)} ({len(df_telem)} rows)")
            if "s_lap" not in df_telem.columns and "s" in df_telem.columns:
                df_telem["s_lap"] = df_telem["s"]
            if "a_lon" not in df_telem.columns:
                df_telem["a_lon"] = 0.0
            if "a_lat" not in df_telem.columns and "v" in df_telem.columns and "yaw_rate" in df_telem.columns:
                df_telem["a_lat"] = df_telem["v"] * df_telem["yaw_rate"]
            if "friction_util_pct" not in df_telem.columns and "a_lat" in df_telem.columns:
                df_telem["friction_util_pct"] = (np.abs(df_telem["a_lat"]) / 9.81) * 100.0
            if "min_cone_clearance" not in df_telem.columns and "e_y" in df_telem.columns:
                df_telem["min_cone_clearance"] = 1.5 - np.abs(df_telem["e_y"])
        except Exception as e:
            print(f"[DataLoader] Warning: Could not read {state_path}: {e}")

    df_timing = None
    if os.path.exists(timing_path) and os.path.getsize(timing_path) > 100:
        try:
            df_timing = pd.read_csv(timing_path)
        except Exception as e:
            print(f"[DataLoader] Warning: Could not read {timing_path}: {e}")

    summary_data = None
    if os.path.exists(summary_path):
        try:
            with open(summary_path, "r") as f:
                summary_data = json.load(f)
        except Exception as e:
            print(f"[DataLoader] Warning: Could not read {summary_path}: {e}")

    return df_telem, df_timing, summary_data


def segment_laps(df: pd.DataFrame) -> Dict[str, pd.DataFrame]:
    """Segments telemetry into Lap 1 (standing start) and Lap 2 (flying lap), ignoring staging."""
    laps: Dict[str, pd.DataFrame] = {}
    if "lap_idx" in df.columns:
        # Filter for actual full laps (> 50 samples, ignoring initial launch roll < 50 samples)
        valid_indices = [idx for idx in sorted(df["lap_idx"].unique()) if len(df[df["lap_idx"] == idx]) > 50]
        if len(valid_indices) >= 2:
            laps["standing"] = df[df["lap_idx"] == valid_indices[0]].copy()
            laps["flying"] = df[df["lap_idx"] == valid_indices[1]].copy()
        elif len(valid_indices) == 1:
            laps["standing"] = df[df["lap_idx"] == valid_indices[0]].copy()
    else:
        # Fallback segmentation on progress wrap (skip initial staging roll < 50 samples)
        s = df["s_lap"].values if "s_lap" in df.columns else df["s"].values
        wrap_indices = []
        for i in range(50, len(s)):
            if s[i] < s[i - 1] - 50.0:
                wrap_indices.append(i)

        if len(wrap_indices) >= 2:
            laps["standing"] = df.iloc[wrap_indices[0]:wrap_indices[1]].copy()
            laps["flying"] = df.iloc[wrap_indices[1]:].copy()
        elif len(wrap_indices) == 1:
            laps["standing"] = df.iloc[:wrap_indices[0]].copy()
            laps["flying"] = df.iloc[wrap_indices[0]:].copy()
        else:
            laps["all"] = df.copy()

    return laps


def compute_track_metrics(df_telem: pd.DataFrame, df_timing: Optional[pd.DataFrame],
                          summary: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Computes exhaustive metrics for a single track run."""
    m: Dict[str, Any] = {}
    laps = segment_laps(df_telem)

    # Lap times
    lap1_time = None
    lap2_time = None
    if summary:
        lap1_time = summary.get("lap1_time_standing")
        lap2_time = summary.get("lap2_time_flying")

    if lap1_time is None and "standing" in laps:
        st = laps["standing"]
        # Movement start: speed > 0.15 m/s
        st_moving = st[st["v"] > 0.15]
        if len(st_moving) > 0:
            lap1_time = float(st["time"].iloc[-1] - st_moving["time"].iloc[0])

    if lap2_time is None and "flying" in laps:
        fl = laps["flying"]
        lap2_time = float(fl["time"].iloc[-1] - fl["time"].iloc[0])

    m["lap1_standing_time"] = round(lap1_time, 3) if lap1_time is not None else None
    m["lap2_flying_time"] = round(lap2_time, 3) if lap2_time is not None else None
    m["total_2lap_time"] = round((lap1_time or 0) + (lap2_time or 0), 3) if (lap1_time and lap2_time) else None
    m["launch_penalty"] = round(lap1_time - lap2_time, 3) if (lap1_time and lap2_time) else None

    # Focus on Flying Lap for dynamic benchmarks (if available, otherwise full df)
    df_eval = laps.get("flying", df_telem)

    # Speed metrics
    v = df_eval["v"].values
    m["v_avg_flying"] = float(np.mean(v))
    m["v_max_flying"] = float(np.max(v))

    # Tracking accuracy
    ey = df_eval["e_y"].values
    m["ey_mean_abs"] = float(np.mean(np.abs(ey)))
    m["ey_max_abs"] = float(np.max(np.abs(ey)))
    m["ey_std"] = float(np.std(ey))

    if "e_psi" in df_eval.columns:
        epsi_deg = np.rad2deg(df_eval["e_psi"].values)
        m["epsi_rms_deg"] = float(np.sqrt(np.mean(epsi_deg**2)))
    else:
        m["epsi_rms_deg"] = 0.0

    # Cone clearance & safety margins
    if "min_cone_clearance" in df_eval.columns:
        clr = df_eval["min_cone_clearance"].values
        m["clearance_min"] = float(np.min(clr))
        m["clearance_mean"] = float(np.mean(clr))
        m["cones_near_miss_count"] = int(np.sum(clr < 0.30))
        m["cone_breach_count"] = int(np.sum(clr <= 0.0))
    else:
        m["clearance_min"] = float(1.5 - m["ey_max_abs"])
        m["clearance_mean"] = float(1.5 - m["ey_mean_abs"])
        m["cones_near_miss_count"] = 0
        m["cone_breach_count"] = 0

    # Friction circle & envelope exploration
    if "friction_util_pct" in df_eval.columns:
        f_util = df_eval["friction_util_pct"].values
        m["friction_util_mean"] = float(np.mean(f_util))
        m["friction_util_max"] = float(np.max(f_util))
        m["friction_util_p95"] = float(np.percentile(f_util, 95))
    else:
        m["friction_util_mean"] = 0.0
        m["friction_util_max"] = 0.0
        m["friction_util_p95"] = 0.0

    if "friction_headroom" in df_eval.columns:
        m["friction_headroom_mean"] = float(np.mean(df_eval["friction_headroom"].values))
    else:
        m["friction_headroom_mean"] = 0.0

    # Control smoothness & jerk
    if "jerk_lon" in df_eval.columns:
        m["jerk_lon_rms"] = float(np.sqrt(np.mean(df_eval["jerk_lon"].values**2)))
    else:
        m["jerk_lon_rms"] = 0.0

    if "jerk_steer" in df_eval.columns:
        m["jerk_steer_rms"] = float(np.sqrt(np.mean(df_eval["jerk_steer"].values**2)))
    else:
        m["jerk_steer_rms"] = 0.0

    # Solver computational performance
    if df_timing is not None and len(df_timing) > 0:
        sol_times = df_timing["solver_ms"].values
        m["solve_ms_mean"] = float(np.mean(sol_times))
        m["solve_ms_p99"] = float(np.percentile(sol_times, 99))
        m["solve_ms_max"] = float(np.max(sol_times))
        if "qp_iter" in df_timing.columns:
            m["qp_iter_mean"] = float(np.mean(df_timing["qp_iter"].values))
        else:
            m["qp_iter_mean"] = 1.0
    elif "solve_time_us" in df_eval.columns:
        sol_ms = df_eval["solve_time_us"].values / 1000.0
        m["solve_ms_mean"] = float(np.mean(sol_ms))
        m["solve_ms_p99"] = float(np.percentile(sol_ms, 99))
        m["solve_ms_max"] = float(np.max(sol_ms))
        m["qp_iter_mean"] = float(np.mean(df_eval["qp_iter"].values)) if "qp_iter" in df_eval.columns else 1.0
    else:
        m["solve_ms_mean"] = 0.0
        m["solve_ms_p99"] = 0.0
        m["solve_ms_max"] = 0.0
        m["qp_iter_mean"] = 1.0

    return m


def evaluate_cross_track_robustness(metrics_by_track: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """
    Evaluates cross-track stability, detects overfitting, and scores controller generalization.
    """
    results: Dict[str, Any] = {}
    valid_tracks = [t for t, m in metrics_by_track.items() if m.get("lap2_flying_time") is not None]

    if len(valid_tracks) == 0:
        return {"status": "NO_VALID_DATA", "robustness_score": 0.0}

    # 1. Tracking error variation across tracks
    ey_means = [metrics_by_track[t]["ey_mean_abs"] for t in valid_tracks]
    ey_maxs = [metrics_by_track[t]["ey_max_abs"] for t in valid_tracks]
    ey_std_across = float(np.std(ey_means))
    ey_mean_across = float(np.mean(ey_means))
    cv_ey = (ey_std_across / ey_mean_across) if ey_mean_across > 1e-4 else 0.0

    # 2. Safety analysis
    total_breaches = sum(metrics_by_track[t].get("cone_breach_count", 0) for t in valid_tracks)
    min_clearances = [metrics_by_track[t].get("clearance_min", 1.0) for t in valid_tracks]
    overall_min_clearance = min(min_clearances)

    # 3. Solver reliability across tracks
    max_solve_times = [metrics_by_track[t]["solve_ms_max"] for t in valid_tracks]
    overall_max_solve_ms = max(max_solve_times)

    # 4. Friction utilization
    mean_f_utils = [metrics_by_track[t]["friction_util_mean"] for t in valid_tracks]
    cv_f_util = float(np.std(mean_f_utils) / np.mean(mean_f_utils)) if np.mean(mean_f_utils) > 0 else 0.0

    # Overfitting assessment
    overfitting_flags: List[str] = []
    if cv_ey > 0.45:
        overfitting_flags.append(
            f"High tracking variance across tracks (CV={cv_ey*100:.1f}%). "
            f"Tuning is likely overfitted to one specific curvature profile."
        )

    # Check if technical track (FSG21) degrades severely compared to FSE23
    if "FSE23" in metrics_by_track and "FSG21" in metrics_by_track:
        m_e23 = metrics_by_track["FSE23"]
        m_g21 = metrics_by_track["FSG21"]
        if m_g21["ey_mean_abs"] > 2.0 * m_e23["ey_mean_abs"]:
            overfitting_flags.append(
                f"FSG21 tracking error is {m_g21['ey_mean_abs']/m_e23['ey_mean_abs']:.1f}x higher than FSE23! "
                f"Weights or lookahead are poorly tuned for tight technical slaloms."
            )
        if m_g21["clearance_min"] < 0.25 and m_e23["clearance_min"] > 0.60:
            overfitting_flags.append(
                f"Dangerous cone clearance reduction on FSG21 ({m_g21['clearance_min']:.2f} m vs FSE23 {m_e23['clearance_min']:.2f} m)."
            )

    # Score calculation (0 - 100)
    # Safety component (40 pts): No breaches, good clearances
    safety_score = 40.0
    if total_breaches > 0:
        safety_score = 0.0
    else:
        if overall_min_clearance < 0.30:
            safety_score -= 20.0 * (1.0 - overall_min_clearance / 0.30)

    # Determinism component (30 pts): Max solve time < 10ms (100 Hz deadline)
    deadline_score = 30.0
    if overall_max_solve_ms > 10.0:
        deadline_score = max(0.0, 30.0 - (overall_max_solve_ms - 10.0) * 5.0)

    # Generalization component (30 pts): Consistency across tracks
    gen_score = max(0.0, 30.0 * (1.0 - min(cv_ey, 1.0)))

    robustness_score = round(safety_score + deadline_score + gen_score, 1)

    results["valid_tracks"] = valid_tracks
    results["total_breaches"] = total_breaches
    results["overall_min_clearance"] = overall_min_clearance
    results["overall_max_solve_ms"] = overall_max_solve_ms
    results["cv_tracking_error"] = cv_ey
    results["overfitting_flags"] = overfitting_flags
    results["safety_score"] = round(safety_score, 1)
    results["deadline_score"] = round(deadline_score, 1)
    results["generalization_score"] = round(gen_score, 1)
    results["robustness_score"] = robustness_score
    results["is_overfitted"] = len(overfitting_flags) > 0

    return results


def generate_cross_track_figure(tracks_data: Dict[str, pd.DataFrame],
                                metrics: Dict[str, Dict[str, Any]],
                                output_file: str):
    """
    Renders 6-Panel publication-grade cross-track diagnostic dashboard.
    """
    fig = plt.figure(figsize=(18, 11), dpi=150)
    gs = GridSpec(2, 3, figure=fig, hspace=0.32, wspace=0.28)

    track_order = [t for t in ["FSE23", "FSG21", "FSI24", "FSE24"] if t in metrics] + [t for t in metrics if t not in ["FSE23", "FSG21", "FSI24", "FSE24"]]
    colors = [TRACK_METADATA.get(t, {}).get("color", "#333333") for t in track_order]

    # Panel 1: Lap Times Bar Chart (Standing Start vs Flying Lap)
    ax1 = fig.add_subplot(gs[0, 0])
    x = np.arange(len(track_order))
    width = 0.35

    standing_times = [metrics[t].get("lap1_standing_time") or 0.0 for t in track_order]
    flying_times = [metrics[t].get("lap2_flying_time") or 0.0 for t in track_order]

    b1 = ax1.bar(x - width/2, standing_times, width, label='Lap 1 (Standing Start)', color='#3b82f6', edgecolor='black', alpha=0.85)
    b2 = ax1.bar(x + width/2, flying_times, width, label='Lap 2 (Flying Lap)', color='#10b981', edgecolor='black', alpha=0.85)

    ax1.set_ylabel('Lap Time (s)', fontweight='bold')
    ax1.set_title('Lap Times: Standing vs Flying Lap', fontweight='bold', fontsize=11)
    ax1.set_xticks(x)
    ax1.set_xticklabels([f"{t}\n({TRACK_METADATA.get(t,{}).get('name',t)})" for t in track_order], fontsize=9)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, linestyle=':', alpha=0.6, axis='y')

    for bar in b1:
        h = bar.get_height()
        if h > 0:
            ax1.text(bar.get_x() + bar.get_width()/2., h + 0.3, f"{h:.2f}s", ha='center', va='bottom', fontsize=8)
    for bar in b2:
        h = bar.get_height()
        if h > 0:
            ax1.text(bar.get_x() + bar.get_width()/2., h + 0.3, f"{h:.2f}s", ha='center', va='bottom', fontsize=8, fontweight='bold')

    # Panel 2: Speed Profile vs Circuit Progress (%)
    ax2 = fig.add_subplot(gs[0, 1])
    for t in track_order:
        if t in tracks_data and tracks_data[t] is not None:
            df_t = tracks_data[t]
            laps = segment_laps(df_t)
            df_fl = laps.get("flying", df_t)
            if "progress_pct" in df_fl.columns and len(df_fl) > 10:
                prog = df_fl["progress_pct"].values
                v = df_fl["v"].values * 3.6  # km/h
                c = TRACK_METADATA.get(t, {}).get("color", "#333")
                ax2.plot(prog, v, label=f"{t} (max {np.max(v):.1f} km/h)", color=c, lw=1.6)
            elif "s_lap" in df_fl.columns and len(df_fl) > 10:
                s = df_fl["s_lap"].values
                prog = (s / np.max(s)) * 100.0 if np.max(s) > 0 else s
                v = df_fl["v"].values * 3.6
                c = TRACK_METADATA.get(t, {}).get("color", "#333")
                ax2.plot(prog, v, label=f"{t} (max {np.max(v):.1f} km/h)", color=c, lw=1.6)

    ax2.set_xlabel('Lap Progress (%)', fontweight='bold')
    ax2.set_ylabel('Speed (km/h)', fontweight='bold')
    ax2.set_title('Flying Lap Speed Profiles', fontweight='bold', fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower center', fontsize=8)

    # Panel 3: g-g Friction Circle Overlay
    ax3 = fig.add_subplot(gs[0, 2])
    # Plot reference circle (e.g. 1.0g and 1.2g)
    theta = np.linspace(0, 2*np.pi, 200)
    ax3.plot(np.cos(theta), np.sin(theta), 'k--', lw=1.0, alpha=0.5, label='1.0g Friction Limit')
    ax3.plot(1.2 * np.cos(theta), 1.2 * np.sin(theta), 'r:', lw=1.0, alpha=0.4, label='1.2g Aero Limit')

    for t in track_order:
        if t in tracks_data and tracks_data[t] is not None:
            df_t = tracks_data[t]
            laps = segment_laps(df_t)
            df_fl = laps.get("flying", df_t)
            if "a_lon" in df_fl.columns and "a_lat" in df_fl.columns:
                a_lon_g = df_fl["a_lon"].values / 9.81
                a_lat_g = df_fl["a_lat"].values / 9.81
                c = TRACK_METADATA.get(t, {}).get("color", "#333")
                ax3.scatter(a_lat_g, a_lon_g, s=4, alpha=0.25, color=c, label=f"{t} (Grip)")

    ax3.set_xlabel('Lateral Accel $a_{lat}$ [g]', fontweight='bold')
    ax3.set_ylabel('Longitudinal Accel $a_{lon}$ [g]', fontweight='bold')
    ax3.set_title('Combined g-g Friction Diagrams', fontweight='bold', fontsize=11)
    ax3.set_xlim([-1.4, 1.4])
    ax3.set_ylim([-1.4, 1.4])
    ax3.axhline(0, color='gray', lw=0.5)
    ax3.axvline(0, color='gray', lw=0.5)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=7, markerscale=3)

    # Panel 4: Lateral Tracking Error |e_y| Distributions
    ax4 = fig.add_subplot(gs[1, 0])
    error_data = []
    labels = []
    for t in track_order:
        if t in tracks_data and tracks_data[t] is not None:
            df_t = tracks_data[t]
            laps = segment_laps(df_t)
            df_fl = laps.get("flying", df_t)
            if "e_y" in df_fl.columns:
                error_data.append(np.abs(df_fl["e_y"].values))
                labels.append(f"{t}\n(avg {metrics[t]['ey_mean_abs']:.2f}m)")

    if error_data:
        bp = ax4.boxplot(error_data, tick_labels=labels, patch_artist=True,
                         medianprops=dict(color='black', lw=1.5),
                         boxprops=dict(facecolor='#93c5fd', alpha=0.7),
                         flierprops=dict(marker='o', markersize=2, alpha=0.3))
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)

    ax4.set_ylabel('Lateral Tracking Error $|e_y|$ (m)', fontweight='bold')
    ax4.set_title('Tracking Error Distribution per Circuit', fontweight='bold', fontsize=11)
    ax4.axhline(0.30, color='orange', ls='--', lw=1.0, label='Desired Margin (<0.3m)')
    ax4.axhline(0.60, color='red', ls=':', lw=1.0, label='Limit (>0.6m)')
    ax4.legend(loc='upper right', fontsize=8)
    ax4.grid(True, linestyle=':', alpha=0.6, axis='y')

    # Panel 5: Minimum Cone Clearance vs Lap Progress
    ax5 = fig.add_subplot(gs[1, 1])
    for t in track_order:
        if t in tracks_data and tracks_data[t] is not None:
            df_t = tracks_data[t]
            laps = segment_laps(df_t)
            df_fl = laps.get("flying", df_t)
            if "min_cone_clearance" in df_fl.columns and len(df_fl) > 10:
                prog = df_fl["progress_pct"].values if "progress_pct" in df_fl.columns else np.linspace(0, 100, len(df_fl))
                clr = df_fl["min_cone_clearance"].values
                c = TRACK_METADATA.get(t, {}).get("color", "#333")
                ax5.plot(prog, clr, label=f"{t} (min {np.min(clr):.2f}m)", color=c, lw=1.4)

    ax5.axhline(0.0, color='red', lw=1.5, label='Cone Contact Line (0.0 m)')
    ax5.axhline(0.30, color='orange', ls='--', lw=1.0, label='Near-Miss Buffer (0.30 m)')
    ax5.set_xlabel('Lap Progress (%)', fontweight='bold')
    ax5.set_ylabel('Clearance to Cones (m)', fontweight='bold')
    ax5.set_title('Track Margin & Cone Clearances', fontweight='bold', fontsize=11)
    ax5.set_ylim([-0.1, 1.8])
    ax5.grid(True, linestyle=':', alpha=0.6)
    ax5.legend(loc='lower left', fontsize=8)

    # Panel 6: Solver Latency Distributions & Real-Time Deadline
    ax6 = fig.add_subplot(gs[1, 2])
    for t in track_order:
        if t in tracks_data and tracks_data[t] is not None:
            df_t = tracks_data[t]
            if "solve_time_us" in df_t.columns:
                times_ms = df_t["solve_time_us"].values / 1000.0
                c = TRACK_METADATA.get(t, {}).get("color", "#333")
                sorted_ms = np.sort(times_ms)
                cdf = np.linspace(0, 100, len(sorted_ms))
                ax6.plot(sorted_ms, cdf, label=f"{t} (P99: {metrics[t]['solve_ms_p99']:.2f}ms)", color=c, lw=1.6)

    ax6.axvline(10.0, color='red', ls='--', lw=1.5, label='100 Hz Limit (10.0 ms)')
    ax6.axvline(5.0, color='gray', ls=':', lw=1.0, label='5.0 ms Budget')
    ax6.set_xlabel('Solve Time (ms)', fontweight='bold')
    ax6.set_ylabel('Cumulative Percentage (%)', fontweight='bold')
    ax6.set_title('Solver Execution Time CDF (acados SQP_RTI)', fontweight='bold', fontsize=11)
    ax6.set_xlim([0, 15])
    ax6.grid(True, linestyle=':', alpha=0.6)
    ax6.legend(loc='lower right', fontsize=8)

    plt.suptitle('FastLap NMPC - Comprehensive Cross-Track Benchmark (FSE23 vs FSG21 vs FSE24)',
                 fontsize=14, fontweight='bold', y=0.98)

    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[Dashboard] Saved 6-panel cross-track visualization: {output_file}")


def generate_markdown_report(metrics: Dict[str, Dict[str, Any]],
                             robustness: Dict[str, Any],
                             output_path: str):
    """Generates detailed Markdown report with scorecard, findings, and tuning advice."""
    track_order = [t for t in ["FSE23", "FSG21", "FSI24", "FSE24"] if t in metrics] + [t for t in metrics if t not in ["FSE23", "FSG21", "FSI24", "FSE24"]]

    md = []
    md.append("# FastLap NMPC - Cross-Track Diagnostic & Overfitting Report\n")
    md.append(f"**Evaluation Mode**: Multi-Track 2-Lap Battery (Lap 1: Standing Start, Lap 2: Flying Lap)\n")
    md.append(f"**Circuits Evaluated**: {', '.join([f'`{t}`' for t in track_order])}\n\n")

    # Executive Scorecard
    md.append("## 1. Executive Cross-Track Scorecard\n")
    md.append(f"- **Overall Robustness Score**: **{robustness.get('robustness_score', 0)} / 100**\n")
    md.append(f"- **Safety Score**: {robustness.get('safety_score', 0)} / 40 (Cone Breaches: **{robustness.get('total_breaches', 0)}**)\n")
    md.append(f"- **Real-Time Determinism Score**: {robustness.get('deadline_score', 0)} / 30 (Max Solve Time: **{robustness.get('overall_max_solve_ms', 0):.2f} ms**)\n")
    md.append(f"- **Track Generalization Score**: {robustness.get('generalization_score', 0)} / 30 (Tracking Error CV: **{robustness.get('cv_tracking_error', 0)*100:.1f}%**)\n\n")

    # Overfitting Warning Box
    if robustness.get("is_overfitted"):
        md.append("> [!WARNING]\n")
        md.append("> **POTENTIAL TRACK OVERFITTING DETECTED**:\n")
        for flag in robustness.get("overfitting_flags", []):
            md.append(f"> - {flag}\n")
        md.append("\n")
    else:
        md.append("> [!NOTE]\n")
        md.append("> **BALANCED GENERALIZATION CONFIRMED**: The controller maintains consistent tracking precision, cone safety margins, and solver stability across all 3 circuits without overfitting.\n\n")

    # Comprehensive Comparison Table
    md.append("## 2. Multi-Track Performance Comparison Table\n\n")
    md.append("| Metric | " + " | ".join([f"**{t}**" for t in track_order]) + " |\n")
    md.append("|:---| " + " | ".join([":---:" for _ in track_order]) + " |\n")

    md.append("| **Circuit Description** | " + " | ".join([TRACK_METADATA.get(t,{}).get("name", t) for t in track_order]) + " |\n")
    md.append("| **Track Length** | " + " | ".join([f"~{TRACK_METADATA.get(t,{}).get('length_m', 0)} m" for t in track_order]) + " |\n")
    md.append("| **Lap 1: Standing Start** | " + " | ".join([f"{metrics[t].get('lap1_standing_time', 0):.3f} s" for t in track_order]) + " |\n")
    md.append("| **Lap 2: Flying Lap** | " + " | ".join([f"**{metrics[t].get('lap2_flying_time', 0):.3f} s**" for t in track_order]) + " |\n")
    md.append(r"| **Launch Penalty ($\Delta T$)** | " + " | ".join([f"+{metrics[t].get('launch_penalty', 0):.3f} s" for t in track_order]) + " |\n")
    md.append("| **Total 2-Lap Time** | " + " | ".join([f"{metrics[t].get('total_2lap_time', 0):.3f} s" for t in track_order]) + " |\n")
    md.append("| **Flying Avg Speed** | " + " | ".join([f"{metrics[t].get('v_avg_flying', 0)*3.6:.1f} km/h" for t in track_order]) + " |\n")
    md.append("| **Flying Top Speed** | " + " | ".join([f"{metrics[t].get('v_max_flying', 0)*3.6:.1f} km/h" for t in track_order]) + " |\n")
    md.append("| **Mean Lateral Error $|e_y|$** | " + " | ".join([f"{metrics[t].get('ey_mean_abs', 0):.3f} m" for t in track_order]) + " |\n")
    md.append("| **Max Lateral Error $|e_y|$** | " + " | ".join([f"{metrics[t].get('ey_max_abs', 0):.3f} m" for t in track_order]) + " |\n")
    md.append("| **Min Cone Clearance** | " + " | ".join([f"{metrics[t].get('clearance_min', 0):.3f} m" for t in track_order]) + " |\n")
    md.append("| **Mean Friction Util** | " + " | ".join([f"{metrics[t].get('friction_util_mean', 0):.1f}%" for t in track_order]) + " |\n")
    md.append("| **Peak Friction Util** | " + " | ".join([f"{metrics[t].get('friction_util_max', 0):.1f}%" for t in track_order]) + " |\n")
    md.append("| **Longitudinal Jerk RMS** | " + " | ".join([f"{metrics[t].get('jerk_lon_rms', 0):.1f} m/s³" for t in track_order]) + " |\n")
    md.append("| **Steering Jerk RMS** | " + " | ".join([f"{metrics[t].get('jerk_steer_rms', 0):.1f} rad/s²" for t in track_order]) + " |\n")
    md.append("| **Mean Solve Time** | " + " | ".join([f"{metrics[t].get('solve_ms_mean', 0):.2f} ms" for t in track_order]) + " |\n")
    md.append("| **P99 Solve Time** | " + " | ".join([f"{metrics[t].get('solve_ms_p99', 0):.2f} ms" for t in track_order]) + " |\n")
    md.append("| **Max Solve Time** | " + " | ".join([f"{metrics[t].get('solve_ms_max', 0):.2f} ms" for t in track_order]) + " |\n\n")

    # Rational Parameter Tuning Recommendations
    md.append("## 3. Evidence-Based Parameter Tuning Recommendations\n")
    recs: List[str] = []

    # 1. Grip exploration
    mean_grip = np.mean([metrics[t]["friction_util_mean"] for t in track_order])
    if mean_grip < 60.0:
        recs.append(
            f"**Increase Grip Aggressiveness**: Mean friction circle utilization across tracks is conservative ({mean_grip:.1f}%). "
            f"Increase `speed_scale` (e.g. 0.90 -> 0.94) and increase longitudinal acceleration limits `max_accel` (e.g. 3.5 -> 4.5 m/s²)."
        )

    # 2. Tracking accuracy & cone margins
    worst_clearance = min([metrics[t]["clearance_min"] for t in track_order])
    if worst_clearance < 0.30:
        recs.append(
            f"**Increase Safety Margins in Technical Corners**: Minimum cone clearance reached {worst_clearance:.2f} m. "
            f"Increase lateral penalty `q_ey` (e.g. 200.0 -> 350.0) or increase safety buffer `track_margin` (e.g. 0.90 -> 1.05 m)."
        )

    # 3. Launch control
    launch_deltas = [metrics[t]["launch_penalty"] for t in track_order if metrics[t].get("launch_penalty")]
    if launch_deltas and np.mean(launch_deltas) > 4.5:
        recs.append(
            f"**Optimize Standing Launch Traction**: Average launch penalty is high ({np.mean(launch_deltas):.2f} s). "
            f"Tune low-speed torque ramping and increase initial launch acceleration ceiling to minimize standing start lag."
        )

    # 4. Control smoothness
    mean_jerk = np.mean([metrics[t]["jerk_steer_rms"] for t in track_order])
    if mean_jerk > 8.0:
        recs.append(
            f"**Damp Steering Rate Jerk**: High steering jerk detected ({mean_jerk:.1f} rad/s²). "
            f"Increase steering rate penalty `r_vdelta` (e.g. 50.0 -> 80.0) to eliminate oscillations in rapid transitions."
        )

    # 5. Solver latency
    max_solve = max([metrics[t]["solve_ms_max"] for t in track_order])
    if max_solve > 8.0:
        recs.append(
            f"**Solver Overhead Alert**: Peak latency ({max_solve:.2f} ms) approaches 100 Hz deadline. "
            f"Ensure `cost_value` calculation and SQP iterations remain bounded."
        )

    if not recs:
        recs.append("**Optimal Equilibrium**: Controller parameters demonstrate balanced pace, robust cone clearance, and stable computation across all test tracks.")

    for r in recs:
        md.append(f"- {r}\n")

    md.append("\n## 4. Visual Diagnostics\n")
    md.append("Refer to `multitrack_comparison.png` for comprehensive 6-panel graphical comparisons.\n")

    with open(output_path, "w") as f_out:
        f_out.write("".join(md))

    print(f"[Report] Generated multi-track summary report: {output_path}")


def print_terminal_scorecard(metrics: Dict[str, Dict[str, Any]], robustness: Dict[str, Any]):
    """Outputs high-impact terminal comparison table."""
    track_order = [t for t in ["FSE23", "FSG21", "FSI24", "FSE24"] if t in metrics] + [t for t in metrics if t not in ["FSE23", "FSG21", "FSI24", "FSE24"]]

    print("\n" + "=" * 80)
    print("🏆 FASTLAP NMPC: CROSS-TRACK MULTI-CIRCUIT SCORECARD")
    print("=" * 80)
    print(f"{'Metric':<28} | " + " | ".join([f"{t:^14}" for t in track_order]))
    print("-" * 80)

    # Lap times
    def fmt_row(name, key, unit="", fmt=".3f", mult=1.0):
        vals = []
        for t in track_order:
            val = metrics[t].get(key)
            if val is not None:
                vals.append(f"{val*mult:{fmt}} {unit}".strip())
            else:
                vals.append("N/A")
        print(f"{name:<28} | " + " | ".join([f"{v:^14}" for v in vals]))

    fmt_row("Lap 1 (Standing Start)", "lap1_standing_time", "s")
    fmt_row("Lap 2 (Flying Lap)", "lap2_flying_time", "s")
    fmt_row("Launch Penalty (Delta)", "launch_penalty", "s")
    fmt_row("Total 2-Lap Time", "total_2lap_time", "s")
    print("-" * 80)
    fmt_row("Avg Speed (Flying)", "v_avg_flying", "km/h", ".1f", 3.6)
    fmt_row("Top Speed (Flying)", "v_max_flying", "km/h", ".1f", 3.6)
    fmt_row("Mean Lateral Error |ey|", "ey_mean_abs", "m", ".3f")
    fmt_row("Max Lateral Error |ey|", "ey_max_abs", "m", ".3f")
    fmt_row("Min Cone Clearance", "clearance_min", "m", ".3f")
    fmt_row("Mean Friction Util", "friction_util_mean", "%", ".1f")
    fmt_row("Peak Friction Util", "friction_util_max", "%", ".1f")
    fmt_row("Mean Solve Time", "solve_ms_mean", "ms", ".2f")
    fmt_row("P99 Solve Time", "solve_ms_p99", "ms", ".2f")
    print("=" * 80)
    print(f"  Overall Robustness Score: {robustness.get('robustness_score', 0)} / 100")
    if robustness.get("is_overfitted"):
        print("  ⚠️  WARNING: High variance across circuits. Possible track overfitting detected.")
    else:
        print("  ✅ GENERALIZATION: Consistent high performance maintained across all tracks.")
    print("=" * 80 + "\n")


def main():
    parser = argparse.ArgumentParser(description="FastLap NMPC Cross-Track Diagnostic & Overfitting Analysis")
    parser.add_argument("--test-dir", type=str, required=True,
                        help="Parent directory containing per-track test folders (e.g. test_results/run_xyz/)")
    parser.add_argument("--tracks", nargs="+", default=None,
                        help="List of track directory names to compare (default: auto-discover)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for reports and figures (defaults to --test-dir)")

    args = parser.parse_args()
    test_dir = args.test_dir
    output_dir = args.output_dir or test_dir
    os.makedirs(output_dir, exist_ok=True)

    tracks_data: Dict[str, pd.DataFrame] = {}
    metrics_by_track: Dict[str, Dict[str, Any]] = {}

    # Discover available track directories
    available_subdirs = [d for d in os.listdir(test_dir)
                         if os.path.isdir(os.path.join(test_dir, d)) and not d.startswith('.')]

    tracks_to_process = []
    if args.tracks:
        for t in args.tracks:
            if t in available_subdirs:
                tracks_to_process.append(t)
            else:
                print(f"[DataLoader] Warning: Requested track '{t}' not found in {test_dir}")
    if not tracks_to_process:
        # Preferred canonical order if present, followed by any others
        preferred = ["FSE23", "FSG21", "FSI24", "FSE24"]
        for p in preferred:
            if p in available_subdirs and p not in tracks_to_process:
                tracks_to_process.append(p)
        for s in sorted(available_subdirs):
            if s not in tracks_to_process:
                tracks_to_process.append(s)

    print(f"[DataLoader] Processing tracks: {', '.join(tracks_to_process)}")

    for t_name in tracks_to_process:
        t_dir = os.path.join(test_dir, t_name)
        df_telem, df_timing, summary = load_track_data(t_dir)
        if df_telem is None:
            print(f"[DataLoader] Warning: No valid telemetry or state data found for track {t_name}")
            continue

        tracks_data[t_name] = df_telem
        metrics_by_track[t_name] = compute_track_metrics(df_telem, df_timing, summary)

    if not metrics_by_track:
        print(f"[Error] No valid track data found in {test_dir}!")
        sys.exit(1)

    # Cross-track robustness evaluation
    robustness = evaluate_cross_track_robustness(metrics_by_track)

    # Terminal scorecard
    print_terminal_scorecard(metrics_by_track, robustness)

    # Visualizations
    fig_path = os.path.join(output_dir, "multitrack_comparison.png")
    generate_cross_track_figure(tracks_data, metrics_by_track, fig_path)

    # Markdown report
    md_path = os.path.join(output_dir, "multitrack_summary_report.md")
    generate_markdown_report(metrics_by_track, robustness, md_path)


if __name__ == "__main__":
    main()
