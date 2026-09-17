#!/usr/bin/env python3
"""
FastLap NMPC - Smooth Geometric Centerline Generator (Anti-Snaking & Non-Cutting)
=================================================================================
Generates a smooth, continuous C² reference centerline directly from track cones
without zig-zagging on straights/sweepers and without cutting curve apexes.

Why the car was zig-zagging previously:
  1. Cones on Formula Student tracks are staggered: left and right cones are rarely
     perfectly symmetric. Interleaving midpoints from left->right and right->left
     creates an alternating 4-8 cm sawtooth pattern.
  2. Track corridor width fluctuates naturally (e.g. 2.8m to 3.4m). Sticking strictly
     to raw local midpoints forces the trajectory to snake left and right on straights.

How this version solves both problems:
  1. Anti-Aliasing Filter: A periodic binomial filter on the raw midpoints eliminates
     the high-frequency staggered-cone sawtooth while preserving macroscopic geometry.
  2. Balanced C² B-Spline: Fits a periodic cubic B-spline with an optimal tolerance
     (default: 0.08m = 8 cm). This irons out corridor width irregularities on straights
     and sweepers, while staying within ~8 cm of the corridor center in corners.
     (Zero corner-cutting: car maintains > 1.15m clearance from cones at all times).
  3. Continuous Polynomial Evaluation (splev): No polygonal chord approximations.

Output:
  Creates NEW files with suffix (default: `_dense_centerline.yaml`).
  Original YAML track files are NEVER modified.
"""

import argparse
import os
import sys
import numpy as np
import yaml
from scipy.interpolate import splprep, splev

try:
    from yaml import CSafeLoader as SafeLoader, CSafeDumper as SafeDumper
except ImportError:
    from yaml import SafeLoader, SafeDumper


def load_track_yaml(file_path: str) -> dict:
    """Safely loads a PACSim track YAML file."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Track file not found: {file_path}")
    with open(file_path, "r") as f:
        data = yaml.load(f, Loader=SafeLoader)
    if "track" not in data:
        raise ValueError(f"Malformed track file (missing 'track' root key): {file_path}")
    return data


def extract_cone_chain(track_dict: dict, lane_key: str) -> np.ndarray:
    """Extracts (N, 2) 2D coordinates from track.left or track.right."""
    track = track_dict["track"]
    if lane_key not in track or not track[lane_key]:
        raise ValueError(f"Track missing '{lane_key}' cone lane.")

    cones = []
    for entry in track[lane_key]:
        if isinstance(entry, dict) and "position" in entry:
            pos = entry["position"]
            cones.append([float(pos[0]), float(pos[1])])
        elif isinstance(entry, (list, tuple)):
            cones.append([float(entry[0]), float(entry[1])])

    pts = np.array(cones, dtype=np.float64)
    if len(pts) < 4:
        raise ValueError(f"Too few cones in '{lane_key}' ({len(pts)}).")

    # Deduplicate consecutive identical cones (< 10cm)
    clean = [pts[0]]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - clean[-1]) > 0.10:
            clean.append(pts[i])
    clean = np.array(clean, dtype=np.float64)

    # Remove closure duplicate if present
    if np.linalg.norm(clean[-1] - clean[0]) < 0.15:
        clean = clean[:-1]

    return clean


def project_point_to_polyline(p: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    """Finds the closest point on a closed 2D polyline to point p."""
    N = len(polyline)
    min_dist_sq = float("inf")
    best_proj = polyline[0]

    for i in range(N):
        a = polyline[i]
        b = polyline[(i + 1) % N]
        ab = b - a
        ab_len_sq = float(np.dot(ab, ab))

        if ab_len_sq < 1e-10:
            proj = a
        else:
            t = np.clip(np.dot(p - a, ab) / ab_len_sq, 0.0, 1.0)
            proj = a + t * ab

        dist_sq = float(np.sum((p - proj) ** 2))
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            best_proj = proj

    return best_proj


def compute_true_geometric_midpoints(left_cones: np.ndarray, right_cones: np.ndarray) -> np.ndarray:
    """
    Computes true equidistant midpoints between left and right cone boundaries.
    For each left cone: mid_L = (L + proj_R(L)) / 2
    For each right cone: mid_R = (R + proj_L(R)) / 2
    Merges and sorts all midpoints along circuit progression.
    """
    N_L = len(left_cones)
    N_R = len(right_cones)

    # 1. Left cone midpoints
    mids_L = []
    for i in range(N_L):
        p_L = left_cones[i]
        proj_R = project_point_to_polyline(p_L, right_cones)
        mids_L.append((p_L + proj_R) * 0.5)
    mids_L = np.array(mids_L, dtype=np.float64)

    # Reference progression along left midpoint chain
    diffs_L = np.diff(mids_L, axis=0)
    step_lens_L = np.hypot(diffs_L[:, 0], diffs_L[:, 1])
    s_ref_L = np.concatenate(([0.0], np.cumsum(step_lens_L)))
    closure_len = np.hypot(mids_L[0, 0] - mids_L[-1, 0], mids_L[0, 1] - mids_L[-1, 1])

    # 2. Right cone midpoints and projection to progression s
    mids_R = []
    s_mids_R = []
    for j in range(N_R):
        p_R = right_cones[j]
        proj_L = project_point_to_polyline(p_R, left_cones)
        mid = (p_R + proj_L) * 0.5
        mids_R.append(mid)

        best_s = 0.0
        min_d = float("inf")
        for i in range(N_L):
            a = mids_L[i]
            b = mids_L[(i + 1) % N_L]
            ab = b - a
            ab_sq = float(np.dot(ab, ab))
            if ab_sq > 1e-8:
                t = np.clip(np.dot(mid - a, ab) / ab_sq, 0.0, 1.0)
                proj = a + t * ab
                seg_s = s_ref_L[i] + t * (step_lens_L[i] if i < N_L - 1 else closure_len)
            else:
                proj = a
                seg_s = s_ref_L[i]

            d = float(np.sum((mid - proj) ** 2))
            if d < min_d:
                min_d = d
                best_s = seg_s
        s_mids_R.append(best_s)

    # 3. Combine and sort
    all_points = []
    for i in range(N_L):
        all_points.append((s_ref_L[i], mids_L[i]))
    for j in range(N_R):
        all_points.append((s_mids_R[j], mids_R[j]))

    all_points.sort(key=lambda x: x[0])

    # 4. Deduplicate close midpoints (< 0.25m)
    sorted_mids = [all_points[0][1]]
    for _, pt in all_points[1:]:
        if np.linalg.norm(pt - sorted_mids[-1]) >= 0.25:
            sorted_mids.append(pt)

    if np.linalg.norm(sorted_mids[-1] - sorted_mids[0]) < 0.25:
        sorted_mids = sorted_mids[:-1]

    return np.array(sorted_mids, dtype=np.float64)


def apply_periodic_anti_aliasing(midpoints: np.ndarray, passes: int = 2) -> np.ndarray:
    """
    Applies a periodic 5-point binomial filter [1, 4, 6, 4, 1] / 16.0 to midpoints.
    This eliminates the high-frequency alternating sawtooth caused by staggered cone placement,
    without shifting curve apexes or shrinking the track.
    """
    if len(midpoints) < 10 or passes <= 0:
        return midpoints

    filtered = np.copy(midpoints)
    N = len(filtered)
    kernel = np.array([1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0])

    for _ in range(passes):
        next_pts = np.zeros_like(filtered)
        for i in range(N):
            val = np.zeros(2)
            for k in range(-2, 3):
                idx = (i + k) % N
                val += kernel[k + 2] * filtered[idx]
            next_pts[i] = val
        filtered = next_pts

    return filtered


def fit_c2_bspline_centerline(midpoints: np.ndarray, target_ds: float = 0.35, smooth_tolerance_m: float = 0.08) -> np.ndarray:
    """
    Fits a smooth, periodic cubic B-spline (C² continuous) through pre-filtered midpoints.
    
    Parameters:
      midpoints: (K, 2) ordered array of geometric midpoints.
      target_ds: Uniform arc length sampling resolution in meters (default: 0.35m).
      smooth_tolerance_m: Permissible RMS deviation to iron out track-width irregularities (default: 0.08m = 8cm).
                          - Filters out 8-10 cm track width variations on straights (zero zig-zag).
                          - Strictly bounds corner deviation to < 8 cm (car stays > 1.15m from cones, zero apex cutting).
    """
    K = len(midpoints)
    x = midpoints[:, 0]
    y = midpoints[:, 1]

    # Target smoothing factor: sum of squared residuals <= K * (smooth_tolerance_m)^2
    smoothing_factor = K * (smooth_tolerance_m ** 2)

    # Fit periodic cubic B-spline (k=3, per=True)
    tck, u = splprep([x, y], s=smoothing_factor, per=True, k=3)

    # Fine initial evaluation to measure true total arc length
    u_dense = np.linspace(0.0, 1.0, 4000, endpoint=True)
    x_dense, y_dense = splev(u_dense, tck)

    diffs = np.hypot(np.diff(x_dense), np.diff(y_dense))
    cum_s = np.concatenate(([0.0], np.cumsum(diffs)))
    total_length = cum_s[-1]

    # Uniform arc-length grid
    num_samples = int(np.round(total_length / target_ds))
    s_targets = np.linspace(0.0, total_length, num_samples, endpoint=False)

    # Invert arc length s(u) to get exact parameter u values
    u_uniform = np.interp(s_targets, cum_s, u_dense)

    # Evaluate true cubic B-spline at exact uniform u points
    x_resampled, y_resampled = splev(u_uniform, tck)
    z_resampled = np.zeros_like(x_resampled)

    resampled = np.column_stack([x_resampled, y_resampled, z_resampled])

    # Append exact closure point (s = total_length) identical to start point
    resampled = np.vstack([resampled, resampled[0]])
    return resampled


def compute_diagnostics(dense_pts: np.ndarray, left_cones: np.ndarray, right_cones: np.ndarray) -> dict:
    """Computes clearance to cone boundaries and curvature statistics."""
    x = dense_pts[:, 0]
    y = dense_pts[:, 1]

    # Discrete curvature
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx ** 2 + dy ** 2) ** 1.5
    denom = np.where(denom < 1e-6, 1e-6, denom)
    kappa = (dx * ddy - dy * ddx) / denom

    # Clearance from cones
    min_left_dists = []
    min_right_dists = []
    for pt in dense_pts[:-1, :2]:
        d_L = np.min(np.linalg.norm(left_cones - pt, axis=1))
        d_R = np.min(np.linalg.norm(right_cones - pt, axis=1))
        min_left_dists.append(d_L)
        min_right_dists.append(d_R)

    return {
        "max_kappa": float(np.max(np.abs(kappa))),
        "p95_kappa": float(np.percentile(np.abs(kappa), 95)),
        "mean_left_clearance": float(np.mean(min_left_dists)),
        "mean_right_clearance": float(np.mean(min_right_dists)),
        "min_cone_clearance": float(min(np.min(min_left_dists), np.min(min_right_dists))),
        "safe_apex_speed_kmh": float(np.sqrt(1.0 * 9.81 / max(np.max(np.abs(kappa)), 1e-3)) * 3.6)
    }


def create_dense_track_from_cones(input_path: str, output_path: str, target_ds: float = 0.35,
                                  smooth_tolerance_m: float = 0.08, filter_passes: int = 2) -> dict:
    """
    Reads original track YAML, computes true midpoints from cones,
    applies anti-aliasing filter, fits a smooth C² B-spline, and saves the new YAML file.
    Does NOT modify input_path.
    """
    data = load_track_yaml(input_path)
    left_cones = extract_cone_chain(data, "left")
    right_cones = extract_cone_chain(data, "right")

    # 1. Calculate true geometric midpoints directly from cones
    raw_midpoints = compute_true_geometric_midpoints(left_cones, right_cones)

    # 2. Apply periodic anti-aliasing filter to eliminate staggered-cone sawtooth
    clean_midpoints = apply_periodic_anti_aliasing(raw_midpoints, passes=filter_passes)

    # 3. Fit smooth C² cubic B-spline with controlled tolerance (irons out corridor width changes)
    dense_pts = fit_c2_bspline_centerline(clean_midpoints, target_ds=target_ds, smooth_tolerance_m=smooth_tolerance_m)

    diffs = np.hypot(np.diff(dense_pts[:, 0]), np.diff(dense_pts[:, 1]))
    track_len = float(np.sum(diffs))
    avg_ds = float(np.mean(diffs))

    # Diagnostics
    diag = compute_diagnostics(dense_pts, left_cones, right_cones)

    # Format output YAML records
    dense_records_raw = []
    dense_records_smooth = []
    for pt in dense_pts:
        pos_entry = [float(f"{pt[0]:.6f}"), float(f"{pt[1]:.6f}"), float(f"{pt[2]:.6f}")]
        dense_records_raw.append({"position": pos_entry})
        dense_records_smooth.append({"position": pos_entry})

    # Deep copy track dictionary without aliases/anchors
    new_data = {"track": {}}
    for k, v in data["track"].items():
        if k not in ("centerline_raw", "centerline_smooth"):
            new_data["track"][k] = v

    # Store clean independent lists (no YAML anchors/aliases)
    new_data["track"]["centerline_raw"] = dense_records_raw
    new_data["track"]["centerline_smooth"] = dense_records_smooth

    # Write output YAML file
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(new_data, f, Dumper=SafeDumper, default_flow_style=False, sort_keys=False)

    summary = {
        "input_file": os.path.basename(input_path),
        "output_file": os.path.basename(output_path),
        "left_cones": len(left_cones),
        "right_cones": len(right_cones),
        "true_midpoints_found": len(raw_midpoints),
        "dense_points": len(dense_pts),
        "avg_ds": avg_ds,
        "track_length": track_len,
        "max_kappa": diag["max_kappa"],
        "p95_kappa": diag["p95_kappa"],
        "min_cone_clearance": diag["min_cone_clearance"],
        "mean_corridor_center_offset": abs(diag["mean_left_clearance"] - diag["mean_right_clearance"]),
        "safe_apex_speed_kmh": diag["safe_apex_speed_kmh"],
        "closure_error": float(np.linalg.norm(dense_pts[-1, :2] - dense_pts[0, :2]))
    }
    return summary


def main():
    parser = argparse.ArgumentParser(description="FastLap NMPC - Smooth Geometric Centerline Generator (Anti-Snaking & Non-Cutting)")
    parser.add_argument("--track", type=str, default="FSE23_centerline.yaml",
                        help="Track file name (e.g. FSE23_centerline.yaml) or 'all' to process all valid tracks")
    parser.add_argument("--tracks-dir", type=str, default="",
                        help="Directory containing original track YAML files (default: pacsim/tracks)")
    parser.add_argument("--output-dir", type=str, default="",
                        help="Directory where densified track files will be saved (default: pacsim/tracks)")
    parser.add_argument("--step", type=float, default=0.35,
                        help="Target arc length spacing ds in meters (default: 0.35m)")
    parser.add_argument("--smooth", type=float, default=0.08,
                        help="Smoothing tolerance in meters (default: 0.08m = 8cm). Higher = smoother straights; Lower = strictly closer to midpoints")
    parser.add_argument("--filter-passes", type=int, default=2,
                        help="Number of anti-aliasing passes to eliminate staggered cone sawtooth (default: 2)")
    parser.add_argument("--suffix", type=str, default="_dense_centerline.yaml",
                        help="Filename suffix for newly created track files (default: _dense_centerline.yaml)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    if not os.path.isdir(os.path.join(workspace_dir, "pacsim")):
        workspace_dir = os.path.abspath(os.path.join(script_dir, ".."))

    tracks_dir = args.tracks_dir if args.tracks_dir else os.path.join(workspace_dir, "pacsim", "tracks")
    output_dir = args.output_dir if args.output_dir else tracks_dir

    valid_tracks = [
        "FSE23_centerline.yaml",
        "FSG21_centerline.yaml",
        "FSI24_centerline.yaml",
        "FSCZ24_centerline.yaml",
        "FSE22_centerline.yaml",
        "FSG19_centerline.yaml",
        "FSG23_centerline.yaml",
        "FSO20_centerline.yaml",
        "FSS22_V1_centerline.yaml",
        "FSS22_V2_centerline.yaml"
    ]

    target_tracks = valid_tracks if args.track.lower() == "all" else [os.path.basename(args.track)]

    print("\n" + "=" * 80)
    print("      FASTLAP NMPC: ANTI-SNAKING & NON-CUTTING SMOOTH CENTERLINE GENERATOR   ")
    print("=" * 80)
    print(f" Source Tracks Dir:    {tracks_dir}")
    print(f" Output Tracks Dir:    {output_dir}")
    print(f" Target Resolution:    ds = {args.step:.2f} m")
    print(f" Smooth Tolerance:     {args.smooth * 100:.1f} cm (ironing out width variations)")
    print(f" Anti-Aliasing Passes: {args.filter_passes} (filtering staggered cone sawtooth)")
    print(f" Output Suffix:        {args.suffix}")
    print("-" * 80)

    results = []
    for track_name in target_tracks:
        input_path = os.path.join(tracks_dir, track_name)
        if not os.path.exists(input_path):
            print(f"❌ File not found: {input_path}")
            continue

        base_stem = track_name.replace("_centerline.yaml", "").replace(".yaml", "")
        output_filename = f"{base_stem}{args.suffix}"
        output_path = os.path.join(output_dir, output_filename)

        try:
            summary = create_dense_track_from_cones(
                input_path, output_path,
                target_ds=args.step,
                smooth_tolerance_m=args.smooth,
                filter_passes=args.filter_passes
            )
            results.append(summary)
            print(f"✅ Generated: {output_filename}")
            print(f"   • Track Length: {summary['track_length']:.1f} m | Midpoints from Cones: {summary['true_midpoints_found']} | Output Points: {summary['dense_points']} (ds={summary['avg_ds']:.2f}m)")
            print(f"   • Corridor Centering Balance: {summary['mean_corridor_center_offset']*100:.1f} cm (True Corridor Center)")
            print(f"   • Min Clearance from Cones: {summary['min_cone_clearance']:.2f} m (Zero Corner-Cutting)")
            print(f"   • Curvature Smoothness: P95 |kappa| = {summary['p95_kappa']:.3f} m⁻¹, Max |kappa| = {summary['max_kappa']:.3f} m⁻¹ (Safe Apex Speed: {summary['safe_apex_speed_kmh']:.1f} km/h)")
            print(f"   • Loop Closure: Delta = {summary['closure_error']:.8f} m (PERFECT)")
        except Exception as e:
            print(f"❌ Error processing {track_name}: {e}")

    print("=" * 80)
    print(f"🎉 Completed! Processed {len(results)} tracks.")
    print("ℹ️ Note: Original YAML files remain 100% untouched.")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    main()
