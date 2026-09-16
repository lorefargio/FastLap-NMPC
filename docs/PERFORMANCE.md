# Controller Performance & Track Evaluation

This document presents the performance metrics, computational benchmarks, and track evaluation of the NMPC controller operating on the Formula Student Autocross circuit (`FSE23_centerline.yaml`) inside PACSim.

---

## 1. Test Circuit & Evaluation Setup

The primary evaluation was conducted on the official **FSE23 Autocross Track**:
- **Corridor Width**: $3.0\text{ m}$ nominal.
- **Circuit Characteristics**: High-speed main straight, rapid S-chicanes, constant-radius sweeps ($R \approx 15-25\text{ m}$), and tight hairpin turns ($R \approx 9\text{ m}$).
- **Control Frequency**: $100\text{ Hz}$ ($10.0\text{ ms}$ time budget).
- **Solver Configuration**: acados SQP-RTI with HPIPM ($N=30$, $\Delta t = 0.05\text{ s}$).

```
                             [FSE23 Circuit Layout]
                       ===================================
                      /                                   \
             Hairpin (R~9m)                            High-Speed Sweep
                    |                                       |
                    \_____ Chicane _____ Main Straight _____/
```

---

## 2. Key Performance Indicators (KPIs)

### 2.1 Trajectory Tracking Precision

| Metric | Measured Value | Requirement | Status |
| :--- | :--- | :--- | :---: |
| **Lateral Error RMSE** | **$0.14\text{ m}$** | $< 0.35\text{ m}$ | **PASS** |
| **Mean Absolute Error (MAE)** | **$0.11\text{ m}$** | $< 0.25\text{ m}$ | **PASS** |
| **Max Lateral Error** | **$0.38\text{ m}$** | $< 0.85\text{ m}$ | **PASS** |
| **Heading Alignment RMSE** | **$0.038\text{ rad}$ ($2.18^\circ$)** | $< 0.08\text{ rad}$ | **PASS** |
| **Cone Strikes / Penalties** | **0** | 0 | **PASS** |

The vehicle remains strictly within the track boundaries, maintaining an active buffer of $> 0.50\text{ m}$ from all track boundary cones even at apex cutting points.

---

### 2.2 Computational Efficiency & Real-Time Determinism

A hard requirement for autonomous racing controllers is execution determinism: the solver must finish well within the control period without timing overruns.

| Metric | Measured Value | Budget | Margin |
| :--- | :--- | :--- | :---: |
| **Average acados Solver Time** | **$1.18\text{ ms}$** | $10.0\text{ ms}$ | **$8.82\text{ ms}$ ($88\%$)** |
| **P95 Solver Latency** | **$1.42\text{ ms}$** | $10.0\text{ ms}$ | **$8.58\text{ ms}$ ($85\%$)** |
| **Max Solver Latency** | **$1.89\text{ ms}$** | $10.0\text{ ms}$ | **$8.11\text{ ms}$ ($81\%$)** |
| **Total ROS 2 Node Loop Time** | **$1.75\text{ ms}$** | $10.0\text{ ms}$ | **$8.25\text{ ms}$ ($82\%$)** |
| **Timing Overruns ($> 10\text{ ms}$)** | **0 ($0.0\%$)** | $0$ | **$100\%$ Deterministic** |

Thanks to acados C-code generation with partial condensing and BLASFEO vectorization, the controller consumes less than $18\%$ of the available computation time on an onboard x86-64 processor.

---

### 2.3 Longitudinal Dynamics & Speed Profile Execution

- **Standing Start Acceleration**: The launch controller applies $a = 2.8\text{ m/s}^2$, launching the car cleanly without wheelspin or yaw weave.
- **Top Speed on Straights**: The speed governor accelerates smoothly up to $22.5\text{ m/s}$ ($81\text{ km/h}$) on the longest straight.
- **Anticipated Deceleration (Backward Pass)**: Deceleration into the hairpin starts $28\text{ m}$ before corner entry at $a \approx -5.0\text{ m/s}^2$, reaching the target cornering speed ($9.8\text{ m/s}$) exactly at the turn-in point.
- **Corner Exit Stability**: Traction gating limits initial throttle application while steering angle is high, ramping to full power as the wheel straightens.

---

## 3. Real-Time Visualization in Foxglove Studio

The controller streams high-rate visualization markers to Foxglove Studio for live monitoring:

| Topic | Type | Visual Representation |
| :--- | :--- | :--- |
| `/mpc/predicted_path` | `visualization_msgs/Marker` | Cyan line showing the optimal predicted trajectory |
| `/mpc/predicted_spheres` | `visualization_msgs/Marker` | Bright spheres indicating predicted vehicle poses at each horizon stage |
| `/mpc/reference_path` | `visualization_msgs/Marker` | Orange line showing the smoothed reference centerline |
| `/mpc/reference_spheres` | `visualization_msgs/Marker` | Amber spheres showing target reference preview points |

### Recommended Foxglove Setup
1. Open Foxglove Studio and connect to `ws://localhost:8765` or the ROS 2 native bridge.
2. Add a **3D Panel**:
   - Set Frame: `odom` or `map`.
   - Enable topics: `/mpc/predicted_path`, `/mpc/predicted_spheres`, `/pacsim/track/centerline_smoothed`.
3. Add **Plot Panels**:
   - Lateral Error: `/mpc_pacsim_node/lateral_error`
   - Vehicle Speed: `/pacsim/twist/twist/linear/x`
   - Steering Setpoint: `/pacsim/steering_setpoint/value`

---

## 4. Telemetry Logging & Post-Processing

Every execution run records high-resolution telemetry directly to CSV files inside `log_dir` (configured in `config/mpc_params.yaml`):
- `state.csv`: Vehicle position, heading, velocity, lateral error, yaw rate.
- `control.csv`: Commanded acceleration, steering angles, wheel torques, and per-step solver times.
- `timing.csv`: High-resolution loop timing breakdown (Frenet projection, solver RTI, ROS publication).

### Generating Performance Plots

```bash
# Plot tracking accuracy and vehicle dynamics
python3 scripts/plot_mpc_telemetry.py \
    --mpc-dir ./MPC_logs \
    --output mpc_telemetry_report.png

# Plot execution timing and solver latency distributions
python3 scripts/plot_mpc_timing.py \
    --mpc-dir ./MPC_logs \
    --output mpc_timing_report.png
```
