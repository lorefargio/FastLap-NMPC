# Parameter Configuration & Tuning Guide

All operational parameters for the controller are centralized in [`config/mpc_params.yaml`]. This file serves as the **Single Source of Truth (SSOT)**: both the acados C code generator (`python_model/generate_c_code.py`) and the runtime ROS 2 node (`mpc_pacsim_node`) read directly from it.

---

## 1. Parameters Reference Table

| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `use_sim_time` | bool | `true` | - | Synchronize with simulation clock (`/clock`) |
| `control_rate` | double | `100.0` | Hz | Outer ROS 2 control loop frequency (10.0 ms budget) |
| `mpc_dt` | double | `0.05` | s | OCP discretization time step ($N=30 \to 1.5\text{ s}$ horizon) |
| `max_torque_per_wheel` | double | `100.0` | Nm | Maximum torque capacity per electric motor |
| `outer_steering_ratio` | double | `0.23` | - | Kinematic ratio from steering wheel to road wheel angle |
| `max_lateral_error` | double | `3.0` | m | Safety threshold for emergency braking shutdown |
| `emergency_stop` | bool | `false` | - | Manual software emergency stop switch |
| `stop_on_trajectory_complete` | bool | `false` | - | Stop vehicle upon completing the reference track |
| `log_dir` | string | `"MPC_logs"` | - | Destination folder for CSV telemetry logs |
| `centerline_topic` | string | `"/pacsim/track/centerline_smoothed"` | - | Input topic for smoothed spline centerline points |
| `speed_limits_csv` | string | `""` | - | Optional CSV table for speed limits (empty = embedded table) |
| `default_track_width` | double | `3.0` | m | Nominal track corridor width |
| `track_margin` | double | `0.90` | m | Clearance buffer from cone boundary ($0.70\text{m half-car} + 0.20\text{m buffer}$) |
| `effective_mu` | double | `1.0` | - | Nominal tire-road friction coefficient |
| `max_accel` | double | `3.5` | $\text{m/s}^2$ | Maximum longitudinal acceleration in OCP |
| `min_accel` | double | `-8.0` | $\text{m/s}^2$ | Maximum longitudinal service braking in OCP |
| `a_brake` | double | `5.8` | $\text{m/s}^2$ | Deceleration rate for backward-pass preview braking profile |
| `speed_scale` | double | `0.90` | - | Multiplier on empirical curvature speed limit curve |
| `max_straight_speed` | double | `25.0` | m/s | Maximum velocity allowed on straightaways |
| `low_speed_threshold` | double | `7.0` | m/s | Speed below which low-speed traction gating is active |
| `high_speed_threshold` | double | `13.0` | m/s | Speed above which full acceleration is unlocked |
| `standing_launch_accel` | double | `2.8` | $\text{m/s}^2$ | High acceleration permitted from standstill on straights |
| `low_speed_max_accel` | double | `0.85` | $\text{m/s}^2$ | Conservative acceleration limit at low speed when steering |
| `corner_exit_steer_derate` | double | `0.75` | - | Torque attenuation proportional to steering angle |
| `max_accel_slew_rate` | double | `6.0` | $\text{m/s}^3$ | Maximum throttle ramp rate (smooth torque onset) |
| `max_decel_slew_rate` | double | `25.0` | $\text{m/s}^3$ | Maximum braking onset rate (rapid emergency braking) |
| `understeer_gradient` | double | `0.0012` | $\text{rad/(m/s}^2\text{)}$ | Dynamic tire slip angle feedforward compensation |

---

## 2. In-Depth Parameter Explanations & Physics

### Category A: Loop Execution & Discretization

#### `control_rate` ($100.0\text{ Hz}$) & `mpc_dt` ($0.05\text{ s}$)
- The controller runs at $100\text{ Hz}$ ($10\text{ ms}$ interval), while the prediction horizon uses $N=30$ steps of $0.05\text{ s}$ ($1.5\text{ s}$ total lookahead).
- Running the outer loop at $100\text{ Hz}$ provides immediate rejection of disturbances and smooth actuator interpolation, while a $0.05\text{ s}$ prediction step gives enough lookahead distance ($37.5\text{ m}$ at $25\text{ m/s}$) to plan through high-speed transitions without exceeding the solver's computational budget.

---

### Category B: Track Boundaries & Cone Safety

#### `track_margin` ($0.90\text{ m}$)
- Formula Student track cones demarcate corridors typically $3.0\text{ m}$ wide.
- The vehicle half-width is $\approx 0.70\text{ m}$. Setting `track_margin = 0.90` provides an active safety buffer of $0.20\text{ m}$ between the outer tire edge and the cones.
- **Tuning Tip**: Increasing to $0.95\text{ m}$ keeps the car strictly away from cones on windy days, while reducing to $0.80\text{ m}$ allows more aggressive apex clipping.

#### `max_lateral_error` ($3.0\text{ m}$)
- If localization jumps or an unexpected sensor fault occurs, causing $e_y > 3.0\text{ m}$, the node safely aborts by publishing zero torque and bringing the car to a controlled stop.

---

### Category C: Curvature Speed Profiling & Braking

#### `speed_scale` ($0.90$) & `max_straight_speed` ($25.0\text{ m/s}$)
- The controller queries an empirical curvature limit table derived from vehicle testing:
  $$v_{\text{safe}}(\kappa) = \min\left(v_{\text{straight}},\, v_{\text{table}}(\kappa)\right) \cdot \text{speed\_scale}$$
- On straight sections ($\kappa \le 0.033755$), the speed governor smoothly transitions from `max_straight_speed * speed_scale` ($22.5\text{ m/s}$ at $0.90$) down to the curve entry speed.
- **Tuning Tip**: `speed_scale = 0.90` guarantees zero cone knockdowns and high stability. Once track geometry is validated, increasing to $0.95$ yields significant lap time reductions.

#### `a_brake` ($5.8\text{ m/s}^2$)
- Since the prediction horizon lookahead is $1.5\text{ s}$ ($\approx 30\text{ m}$ at speed), braking from $22\text{ m/s}$ down to $9\text{ m/s}$ for a hairpin requires $\approx 35\text{ m}$.
- The `SpeedGovernor` performs a **backward-pass braking integration**:
  $$v[i] = \min\left(v_{\text{geom}}[i],\, \sqrt{v[i+1]^2 + 2 \cdot a_{\text{brake}} \cdot \Delta s}\right)$$
- Setting `a_brake = 5.8\text{ m/s}^2$ triggers the deceleration phase $25\text{ m}$ before corner entry, preventing late-braking lockups and understeer off the track.

---

### Category D: Launch Control & Traction Gating

#### Standing Launch vs. Hairpin Exit
- At low speeds ($v < \text{low\_speed\_threshold} = 7.0\text{ m/s}$), two conflicting requirements exist:
  1. **Standing Start**: When the car starts from standstill on a straight line ($|\delta| < 0.05\text{ rad}$), high acceleration (`standing_launch_accel = 2.8\text{ m/s}^2`) is required to minimize launch time.
  2. **Hairpin Exit**: When exiting a slow tight corner with the steering turned ($|\delta| > 0.15\text{ rad}$), full acceleration causes severe torque yaw moments and vehicle oscillations.
- **Traction Gating Formula**:
  $$a_{\text{eff\_max}} = \operatorname{lerp}\left(a_{\text{low}},\, \text{max\_accel},\, \frac{v - v_{\text{low}}}{v_{\text{high}} - v_{\text{low}}}\right)$$
  where $a_{\text{low}}$ dynamically derates based on steering wheel angle:
  $$a_{\text{low}} = \text{low\_speed\_max\_accel} \cdot \left(1.0 - \text{corner\_exit\_steer\_derate} \cdot \frac{|\delta|}{\delta_{\text{max}}}\right)$$
  This completely eliminates weave and instability when accelerating out of tight corners.

---

### Category E: Dynamic Compensation & Actuator Protection

#### `max_accel_slew_rate` ($6.0\text{ m/s}^3$) & `max_decel_slew_rate` ($25.0\text{ m/s}^3$)
- Slew rate limiting prevents instantaneous jumps in acceleration commands between consecutive $100\text{ Hz}$ iterations.
- Throttle ramp is capped at $6.0\text{ m/s}^3$ to protect the inverter and tire contact patch.
- Braking ramp is set to $25.0\text{ m/s}^3$ to ensure rapid, uninhibited stopping performance when the controller commands a deceleration.

#### `understeer_gradient` ($0.0012\text{ rad/(m/s}^2\text{)}$)
- The kinematic bicycle model assumes zero tire lateral compliance. In reality, pneumatic tires develop lateral force through slip angles:
  $$\alpha_f - \alpha_r \approx K_{us} \cdot a_{\text{lat}} = K_{us} \cdot v^2 \kappa$$
- The node injects feedforward slip angle compensation directly to the final steering command:
  $$\delta_{\text{cmd}} = \delta_{\text{opt}} + K_{us} \cdot v^2 \kappa_{\text{ref}}$$
- This eliminates understeer drift on high-speed sweeps, keeping the car centered on the desired trajectory.

---

## 3. Systematic Tuning Workflow

When adapting the controller to a new vehicle or track:

1. **Step 1: Track Clearance Baseline**
   - Start with conservative parameters: `speed_scale: 0.80`, `max_straight_speed: 18.0`, `track_margin: 0.95`.
   - Verify the vehicle completes clean laps with zero cone strikes.
2. **Step 2: Corner Entry Deceleration**
   - If the vehicle enters hairpins too fast or overshoots the apex, increase `a_brake` (e.g. from $5.0$ to $5.8\text{ m/s}^2$) so braking initiates earlier along the straight.
3. **Step 3: Corner Exit Stability**
   - If the car weaves when exiting hairpins, lower `low_speed_max_accel` (e.g., $0.85 \to 0.70$) or increase `corner_exit_steer_derate` ($0.75 \to 0.85$).
4. **Step 4: High-Speed Tracking Accuracy**
   - If the car drifts wide in high-speed sweeps ($v > 15\text{ m/s}$), slightly increase `understeer_gradient` (e.g., $0.0012 \to 0.0015$).
5. **Step 5: Pushing the Envelope**
   - Once tracking is centered, raise `speed_scale` (from $0.90$ to $0.95$ or $0.98$) and `max_straight_speed` (e.g., $25.0 \to 27.0\text{ m/s}$) to achieve competitive lap times.
   - Regenerate C code (`python3 python_model/generate_c_code.py`) and rebuild the package with `colcon build` whenever OCP box constraints or discretization steps are updated.
