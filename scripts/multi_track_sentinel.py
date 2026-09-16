#!/usr/bin/env python3
"""
multi_track_sentinel.py
Universal Multi-Track Lap Sentinel for FastLap NMPC.

Accurately tracks 2 FULL LAPS across 3 Start/Finish Line Gate Crossings:
  - Starting Grid -> Start Line: Staging roll (~2-6 meters from spawn box).
  - Gate Crossing #1 (Passaggio 1): Breaks start line timer -> Begins Lap 1.
  - Gate Crossing #2 (Passaggio 2): Completes Lap 1 (Standing Start Lap). Begins Lap 2.
  - Gate Crossing #3 (Passaggio 3): Completes Lap 2 (Flying Lap).

Upon completing 2 full laps (3 gate crossings):
1. Calls /pacsim/finish_signal to generate PACSim report and trigger clean shutdown.
2. Writes structured laps_summary.json with high-precision lap timings.
3. Outputs terminal scorecard.
"""

import os
import sys
import time
import json
import math
import argparse
import subprocess
from typing import Optional, Dict, Any, List


def call_pacsim_finish_signal(timeout_sec: float = 5.0) -> bool:
    """Calls ROS 2 /pacsim/finish_signal service to cleanly finish simulation."""
    print("[Sentinel] Triggering /pacsim/finish_signal ROS 2 service...")
    try:
        cmd = ["ros2", "service", "call", "/pacsim/finish_signal", "std_srvs/srv/Empty"]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_sec, text=True)
        if res.returncode == 0:
            print("[Sentinel] PACSim finish_signal acknowledged successfully.")
            return True
        else:
            print(f"[Sentinel] Service call returned non-zero ({res.returncode}): {res.stderr.strip()}")
            return False
    except FileNotFoundError:
        print("[Sentinel] Warning: 'ros2' command not in PATH. Skipping service call.")
        return False
    except subprocess.TimeoutExpired:
        print("[Sentinel] Warning: ros2 service call timed out.")
        return False
    except Exception as e:
        print(f"[Sentinel] Warning: Could not call finish_signal: {e}")
        return False


def run_sentinel(log_dir: str, track_name: str, target_laps: int = 2,
                 output_dir: Optional[str] = None, timeout_sec: float = 240.0,
                 trigger_service: bool = True) -> int:
    """
    Tails telemetry file and records 3 gate crossings (2 full laps).
    """
    if output_dir is None:
        output_dir = log_dir
    os.makedirs(output_dir, exist_ok=True)
    summary_path = os.path.join(output_dir, "laps_summary.json")

    telemetry_file = os.path.join(log_dir, "mpc_telemetry.csv")
    state_file = os.path.join(log_dir, "mpc_state.csv")

    target_crossings = target_laps + 1  # 2 laps requires 3 gate passes

    print("=" * 70)
    print(f"🏎️  FASTLAP NMPC UNIVERSAL LAP SENTINEL")
    print(f"   Track:            {track_name}")
    print(f"   Target Laps:      {target_laps} full laps ({target_crossings} line crossings)")
    print(f"   Lap 1:            Standing Start (Gate 1 -> Gate 2)")
    print(f"   Lap 2:            Flying Lap     (Gate 2 -> Gate 3)")
    print(f"   Log Directory:    {log_dir}")
    print(f"   Output JSON:      {summary_path}")
    print(f"   Watchdog Timeout: {timeout_sec:.1f} s")
    print("=" * 70)

    # 1. Wait for log file creation
    print("[Sentinel] Waiting for telemetry log stream to initialize...")
    wait_start = time.time()
    active_file = None
    file_mode = "telemetry"

    while time.time() - wait_start < 45.0:
        if os.path.exists(telemetry_file) and os.path.getsize(telemetry_file) > 50:
            active_file = telemetry_file
            file_mode = "telemetry"
            break
        elif os.path.exists(state_file) and os.path.getsize(state_file) > 50:
            active_file = state_file
            file_mode = "state"
            break
        time.sleep(0.2)

    if not active_file:
        print(f"[Sentinel] ERROR: No telemetry files detected in {log_dir} after 45s!")
        failure_data = {
            "track": track_name,
            "status": "LOG_NOT_FOUND",
            "completed_laps": 0,
            "error": f"No telemetry files found in {log_dir}"
        }
        with open(summary_path, "w") as f:
            json.dump(failure_data, f, indent=2)
        return 1

    print(f"[Sentinel] Connected to live log: {os.path.basename(active_file)} ({file_mode} mode)")

    # 2. Timing and crossing state variables
    movement_started = False
    t_launch_sim: Optional[float] = None
    t_gate_pass1: Optional[float] = None
    t_gate_pass2: Optional[float] = None
    t_gate_pass3: Optional[float] = None

    lap1_time: Optional[float] = None
    lap2_time: Optional[float] = None

    current_gate_crossings = 0
    last_lap_idx_seen = 0
    last_s = 0.0
    max_s_seen = 0.0

    header_parsed = False
    col_indices: Dict[str, int] = {}

    f = open(active_file, "r")
    start_wall_time = time.time()

    last_t_sim = 0.0

    try:
        while True:
            # Check watchdog timeout
            elapsed_wall = time.time() - start_wall_time
            if elapsed_wall > timeout_sec:
                print(f"\n[Sentinel] ERROR: Timeout of {timeout_sec:.1f}s exceeded!")
                break

            raw_line = f.readline()
            if not raw_line:
                time.sleep(0.01)
                continue

            # Ensure line was completely written and flushed (ends with newline)
            if not raw_line.endswith("\n") and not raw_line.endswith("\r"):
                f.seek(f.tell() - len(raw_line))
                time.sleep(0.01)
                continue

            line = raw_line.strip()
            if not line:
                continue

            # Parse CSV header
            if not header_parsed:
                headers = [h.strip() for h in line.split(",")]
                col_indices = {name: idx for idx, name in enumerate(headers)}
                header_parsed = True
                print(f"[Sentinel] Log header parsed with {len(col_indices)} fields.")
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) != len(col_indices):
                continue

            try:
                t_sim = float(parts[col_indices.get("time", col_indices.get("t", 0))])
                # Simulation time must be strictly non-negative and monotonic
                if t_sim < 0.0 or t_sim < last_t_sim:
                    continue
                last_t_sim = t_sim

                v = float(parts[col_indices["v"]]) if "v" in col_indices else 0.0

                # Detect vehicle launch from the staging box
                if not movement_started:
                    if v > 0.15:
                        movement_started = True
                        t_launch_sim = t_sim
                        print(f"🟢 [LAUNCH DETECTED] Car launched from starting box at sim_time = {t_sim:.3f} s")
                    else:
                        continue

                # Determine current crossing based on lap_idx or s wrap-around
                detected_crossing = current_gate_crossings
                if "lap_idx" in col_indices and col_indices["lap_idx"] < len(parts):
                    try:
                        detected_crossing = int(float(parts[col_indices["lap_idx"]]))
                    except ValueError:
                        pass
                else:
                    # Fallback wrap-around detection on arc length s
                    s_val = float(parts[col_indices.get("s", col_indices.get("s_lap", 0))])
                    if s_val > max_s_seen:
                        max_s_seen = s_val
                    if max_s_seen > 100.0 and s_val < last_s - 50.0:
                        detected_crossing = current_gate_crossings + 1
                        max_s_seen = 0.0
                    last_s = s_val

                # Process gate crossing transitions with minimum lap debounce (>= 10.0s)
                if detected_crossing > current_gate_crossings:
                    # Debounce: a full lap in Formula Student takes at least 10s
                    if detected_crossing == 2 and t_gate_pass1 is not None and (t_sim - t_gate_pass1 < 10.0):
                        continue
                    if detected_crossing >= 3 and t_gate_pass2 is not None and (t_sim - t_gate_pass2 < 10.0):
                        continue

                    current_gate_crossings = detected_crossing

                    # PASS 1: Reaching the start/finish line from the launch box
                    if current_gate_crossings == 1:
                        t_gate_pass1 = t_sim
                        staging_dt = t_sim - (t_launch_sim if t_launch_sim is not None else t_sim)
                        print(f"🏁 [GATE PASS 1 / {target_crossings}] Reached Start Line: sim_time = {t_sim:.3f} s (Launch roll: {staging_dt:.2f} s). Lap 1 timing started!")

                    # PASS 2: Completing Lap 1 (Standing Start Lap)
                    elif current_gate_crossings == 2:
                        t_gate_pass2 = t_sim
                        if t_gate_pass1 is not None:
                            lap1_time = t_gate_pass2 - t_gate_pass1
                        else:
                            lap1_time = t_gate_pass2 - (t_launch_sim if t_launch_sim is not None else 0.0)
                        from_launch_dt = t_gate_pass2 - (t_launch_sim if t_launch_sim is not None else 0.0)
                        print(f"🏁 [GATE PASS 2 / {target_crossings} - LAP 1 COMPLETE: STANDING START] Time: {lap1_time:.3f} s (from box: {from_launch_dt:.3f} s). Lap 2 Flying Lap begins!")

                    # PASS 3: Completing Lap 2 (Flying Lap)
                    elif current_gate_crossings >= 3:
                        t_gate_pass3 = t_sim
                        if t_gate_pass2 is not None:
                            lap2_time = t_gate_pass3 - t_gate_pass2
                        else:
                            lap2_time = 0.0
                        print(f"🏁 [GATE PASS 3 / {target_crossings} - LAP 2 COMPLETE: FLYING LAP]     Time: {lap2_time:.3f} s (sim_time: {t_sim:.3f} s).")
                        break

            except (ValueError, IndexError):
                continue

    finally:
        f.close()

    # Completed full laps evaluation
    completed_laps = max(0, current_gate_crossings - 1)
    success = (current_gate_crossings >= target_crossings) and (lap1_time is not None) and (lap2_time is not None)
    status = "COMPLETED" if success else "INCOMPLETE"

    total_2lap_time = (lap1_time or 0.0) + (lap2_time or 0.0) if success else 0.0
    launch_penalty = (lap1_time - lap2_time) if (lap1_time and lap2_time) else 0.0
    lap1_from_box = (t_gate_pass2 - t_launch_sim) if (t_gate_pass2 is not None and t_launch_sim is not None) else None

    summary_data: Dict[str, Any] = {
        "track": track_name,
        "status": status,
        "gate_crossings": current_gate_crossings,
        "target_gate_crossings": target_crossings,
        "completed_laps": completed_laps,
        "target_laps": target_laps,
        "t_launch_sim": round(t_launch_sim, 3) if t_launch_sim is not None else None,
        "t_gate_pass1": round(t_gate_pass1, 3) if t_gate_pass1 is not None else None,
        "t_gate_pass2": round(t_gate_pass2, 3) if t_gate_pass2 is not None else None,
        "t_gate_pass3": round(t_gate_pass3, 3) if t_gate_pass3 is not None else None,
        "lap1_time_standing": round(lap1_time, 3) if lap1_time is not None else None,
        "lap1_time_from_box": round(lap1_from_box, 3) if lap1_from_box is not None else None,
        "lap2_time_flying": round(lap2_time, 3) if lap2_time is not None else None,
        "total_2lap_time": round(total_2lap_time, 3) if success else None,
        "launch_penalty_delta": round(launch_penalty, 3) if (lap1_time and lap2_time) else None,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }

    with open(summary_path, "w") as f_out:
        json.dump(summary_data, f_out, indent=2)

    # Trigger clean PACSim finish
    if trigger_service and success:
        call_pacsim_finish_signal()

    # Terminal Scorecard
    print("\n" + "=" * 70)
    print(f"📋 FASTLAP NMPC MULTI-TRACK SENTINEL: {track_name}")
    print("=" * 70)
    print(f"  Status:                   {status}")
    print(f"  Gate Crossings:           {current_gate_crossings} / {target_crossings}")
    print(f"  Completed Full Laps:      {completed_laps} / {target_laps}")
    if lap1_time is not None:
        print(f"  Lap 1 (Standing Start):   {lap1_time:.3f} s" + (f" (from box: {lap1_from_box:.3f} s)" if lap1_from_box else ""))
    if lap2_time is not None:
        print(f"  Lap 2 (Flying Lap):       {lap2_time:.3f} s")
    if success:
        print(f"  Launch Delta (Standing-Fly): {launch_penalty:+.3f} s")
        print(f"  Total 2-Lap Time:         {total_2lap_time:.3f} s")
    print(f"  Summary saved to:         {summary_path}")
    print("=" * 70 + "\n")

    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description="Universal Multi-Track Lap Sentinel for FastLap NMPC (3 Gate Passes)")
    parser.add_argument("--log-dir", type=str, default="/workspace/MPC_logs",
                        help="Directory where mpc_telemetry.csv is written")
    parser.add_argument("--track", type=str, default="FSE23",
                        help="Track identifier (e.g. FSE23, FSG21, FSE24)")
    parser.add_argument("--target-laps", type=int, default=2,
                        help="Number of target full laps (default: 2, corresponds to 3 gate passes)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Directory to save laps_summary.json (defaults to log-dir)")
    parser.add_argument("--timeout", type=float, default=240.0,
                        help="Maximum execution timeout in seconds (default: 240.0)")
    parser.add_argument("--no-service", action="store_true",
                        help="Do not call /pacsim/finish_signal ROS 2 service upon completion")

    args = parser.parse_args()
    ret = run_sentinel(
        log_dir=args.log_dir,
        track_name=args.track,
        target_laps=args.target_laps,
        output_dir=args.output_dir,
        timeout_sec=args.timeout,
        trigger_service=not args.no_service
    )
    sys.exit(ret)


if __name__ == "__main__":
    main()
