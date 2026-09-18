# Parameter Configuration & Tuning Guide

All operational parameters for the controller are centralized in [`config/mpc_params.yaml`](../config/mpc_params.yaml). This file serves as the **Single Source of Truth (SSOT)**: both the acados C code generator (`python_model/generate_c_code.py`) and the runtime ROS 2 node (`mpc_pacsim_node`) read directly from it.

---

## 1. Parameters Reference Table

| Parameter | Type | Current Race Value | Unit | Description |
| :--- | :--- | :---: | :--- | :--- |
| `use_sim_time` | bool | `true` | - | Synchronize with simulation clock (`/clock`) |
| `control_rate` | double | `100.0` | Hz | Outer ROS 2 control loop frequency (10.0 ms budget) |
| `mpc_dt` | double | `0.05` | s | OCP discretization time step ($N=30 \to 1.5\text{ s}$ horizon preview) |
| `max_torque_per_wheel` | double | `100.0` | Nm | Maximum torque capacity per electric motor |
| `outer_steering_ratio` | double | `0.23` | - | Kinematic ratio from steering wheel to road wheel angle |
| `max_lateral_error` | double | `3.0` | m | Safety threshold for emergency braking shutdown |
| `emergency_stop` | bool | `false` | - | Manual software emergency stop switch |
| `stop_on_trajectory_complete` | bool | `false` | - | Stop vehicle upon completing the reference track |
| `log_dir` | string | `"MPC_logs"` | - | Destination folder for CSV telemetry logs |
| `centerline_topic` | string | `"/pacsim/track/centerline_smoothed"` | - | Input topic for smoothed spline centerline points |
| `speed_limits_csv` | string | `""` | - | Optional CSV table for speed limits (empty = embedded lookup table) |
| `default_track_width` | double | `3.0` | m | Nominal track corridor width |
| `track_margin` | double | `0.80` | m | Safety clearance from cones (0.70m half-car + 0.10m buffer, unlocks ±0.70m corridor) |
| `effective_mu` | double | `1.0` | - | Nominal tire-road friction coefficient |
| `max_accel` | double | `4.8` | m/s² | Maximum longitudinal acceleration in OCP & powertrain |
| `min_accel` | double | `-8.0` | m/s² | Maximum longitudinal service braking in OCP |
| `a_brake` | double | `5.0` | m/s² | Deceleration rate for backward-pass preview braking profile |
| `speed_scale` | double | `1.00` | - | Multiplier on empirical curvature speed limit curve (100% limit) |
| `max_straight_speed` | double | `25.0` | m/s | Maximum velocity allowed on straightaways (90.0 km/h) |
| `low_speed_threshold` | double | `6.0` | m/s | Speed below which low-speed traction gating is active |
| `high_speed_threshold` | double | `12.0` | m/s | Speed above which full acceleration is unlocked |
| `standing_launch_accel` | double | `4.8` | m/s² | Maximum forward acceleration allowed during standing launch |
| `low_speed_max_accel` | double | `2.20` | m/s² | Progressive baseline acceleration on tight corner exits |
| `corner_exit_steer_derate` | double | `0.55` | - | Proportional torque derating factor based on steering angle |
| `max_accel_slew_rate` | double | `9.0` | m/s³ | Maximum longitudinal acceleration rise rate (rapid throttle pickup) |
| `max_decel_slew_rate` | double | `25.0` | m/s³ | Maximum braking onset rate (instant emergency stopping power) |
| `understeer_gradient` | double | `0.0008` | rad/(m/s²) | Dynamic tire slip feedforward angle compensation |

---

## 2. In-Depth Parameter Explanations & Physics

### Category A: Loop Execution & Discretization

#### `control_rate` (100.0 Hz) & `mpc_dt` (0.05 s)
- The outer ROS 2 control loop operates at **100 Hz** (10.0 ms interval), while the acados prediction horizon uses $N=30$ stages with $\Delta t = 0.05\text{ s}$ (1.5 s total lookahead).
- Running the outer loop at 100 Hz ensures sub-second disturbance rejection and smooth command streaming to the PACSim powertrain.
- A 1.5 s preview horizon allows the vehicle to "see" up to 37.5 m ahead at 25 m/s (90 km/h), enabling optimal corner-entry trail braking and early apex targeting.

---

### Category B: Autonomous Racing Corridor & Cone Safety

#### `track_margin` (0.80 m)
- Formula Student tracks have nominal widths of 3.0 m ($w_l \approx 1.50\text{ m}, w_r \approx 1.50\text{ m}$).
- The vehicle total width is $\approx 1.40\text{ m}$ (half-width 0.70 m).
- Setting `track_margin = 0.80` reserves an active geometric safety cushion of 0.10 m between the outer tire edge and the cones.
- This unlocks **±0.70 m of lateral freedom**, allowing the NMPC to autonomously widen entry angles, cut apexes, and run wide on exits without knocking down cones.
- Telemetry across all three benchmark circuits confirms that with this setting, the minimum physical cone clearance is consistently **$\ge 0.75\text{ m}$**, yielding **0 cone strikes**.

#### Stage-Dependent Lateral Bounds (`setStageLateralBounds`)
- At every horizon stage $k \in [1, N-1]$, the controller evaluates the local corridor bounds from the cubic splines fitted to the track cones:

  $$
  e_{y,\min,k} = -(w_{r,k} - \text{margin}), \quad e_{y,\max,k} = w_{l,k} - \text{margin}
  $$

- In acados, these bounds are enforced as soft constraints with high quadratic penalties ($Z_l = 3000, Z_u = 3000$). This guarantees QP feasibility even during sharp evasive maneuvers while strictly repelling the car from cone limits.

#### `max_lateral_error` (3.0 m)
- Safety supervisor limit. If vehicle tracking error exceeds 3.0 m due to localization loss or physical collision, the node commands immediate emergency zero-torque braking and safely halts.

---

### Category C: Curvature Speed Profiling & Backward Pass

#### `speed_scale` (1.00) & `max_straight_speed` (25.0 m/s)
- The `SpeedGovernor` module queries empirical vehicle limit curves (`Velocità limite.xlsx`) to determine safe cornering velocities as a function of path curvature $\kappa$:

  $$
  v_{\text{safe}}(\kappa) = \min\left(v_{\text{straight}},\, v_{\text{empirical}}(\kappa_{\text{eff}})\right) \cdot \text{speed\_scale}
  $$

- **Corridor-Aware Effective Curvature**:

  $$
  \kappa_{\text{eff}} = \frac{|\kappa|}{1.0 + |\kappa| \cdot W_{\text{free}}}
  $$

  where $W_{\text{free}} = \min(w_l, w_r) - \text{margin}$. This accounts for the larger effective turning radius $R_{\text{eff}} = R_{\text{centerline}} + W_{\text{free}}$ achievable by an out-in-out racing line.
- Setting `speed_scale = 1.00` unlocks the vehicle's full empirical grip limit.

#### `a_brake` (5.0 m/s²)
- High-speed straights allow speeds up to 25.0 m/s (90 km/h), while hairpin apex speeds drop to $\approx 9.5\text{ m/s}$. Braking over this range requires $\approx 27\text{ m}$.
- The `SpeedGovernor` performs a **backward-pass dynamic braking integration** over an extended preview lookahead of up to 65 m:

  $$
  v[i] = \min\left(v[i],\, \sqrt{v[i+1]^2 + 2 \cdot a_{\text{brake}} \cdot \Delta s}\right)
  $$

- Setting `a_brake = 5.0 m/s²` initiates straight-line braking well before corner turn-in, preventing understeer push off the circuit.

---

### Category D: Traction Control & Friction Circle Protection

#### Kamm Friction Circle Grip Capping
- To prevent snap oversteer / spinouts on corner exits when the car commands high throttle while tires are under heavy lateral cornering load, `mpc_pacsim_node` applies a real-time Kamm circle envelope:

  $$
  a_{\text{lat}} \approx \frac{v^2}{L} \tan |\delta|
  $$

  $$
  a_{\text{lon,kamm}} = \sqrt{\max\left(0.4,\, (\mu g \cdot 0.92)^2 - a_{\text{lat}}^2\right)}
  $$

  $$
  a_{\text{eff,max}} = \min\left(a_{\text{eff,max}},\, a_{\text{lon,kamm}}\right)
  $$

- When cornering hard at 1.8 g, longitudinal acceleration is derated to safeguard lateral grip. As the steering straightens on corner exit, $a_{\text{eff,max}} \to 4.8\text{ m/s}^2$, launching the car at full motor power.

#### Dynamic Launch Governor (`standing_launch_accel = 4.8`, `low_speed_max_accel = 2.20`)
- **Standing Start ($v < 1.0\text{ m/s}, |\delta| < 0.08\text{ rad}$)**: Commands full 4.8 m/s² launch drive, cutting standing start lap penalties to $< 0.9\text{ s}$.
- **Hairpin Exit**: When navigating tight curves at low speed, acceleration smoothly blends according to steering angle:

  $$
  \text{traction\_factor} = \operatorname{clamp}\left(1.0 - \text{corner\_exit\_steer\_derate} \cdot \left(\frac{|\delta|}{\delta_{\max}}\right)^2,\, 0.30,\, 1.0\right)
  $$

  This completely eliminates yaw wobble and chatter on slow corner exits.

---

### Category E: Actuator Dynamics & Slew Rate Limiting

#### `max_accel_slew_rate` (8.0 m/s³) & `max_decel_slew_rate` (25.0 m/s³)
- Slew rate filtering protects the electric motors, inverters, and tire contact patches from shock loads:

  $$
  a_{\text{cmd},k} \in \left[a_{\text{cmd},k-1} - \text{max\_decel\_slew} \cdot \Delta t,\, a_{\text{cmd},k-1} + \text{max\_accel\_slew} \cdot \Delta t\right]
  $$

- Acceleration rises at up to 8.0 m/s³ for responsive torque build-up.
- Deceleration is permitted up to 25.0 m/s³ to ensure rapid stopping response during emergency braking.

#### `understeer_gradient` (0.0008 rad/(m/s²))
- Compensates for tire cornering compliance and pneumatic trail at high lateral loads:

  $$
  \delta_{\text{dyn}} = K_{\text{us}} \cdot v^2 \cdot \kappa
  $$

- Blended in progressively when $v > 7.0\text{ m/s}$ and $|a_{\text{lat}}| > 2.5\text{ m/s}^2$, preventing understeer push at the limits of adhesion.
