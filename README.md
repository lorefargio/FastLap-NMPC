# FastLap NMPC: Autonomous Racing Model Predictive Contouring Control in Frenet Frame

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg?logo=python)](https://www.python.org/)
[![acados](https://img.shields.io/badge/acados-SQP--RTI-brightgreen.svg)](https://docs.acados.org/)
[![CasADi](https://img.shields.io/badge/CasADi-3.6+-orange.svg)](https://web.casadi.org/)

High-performance, real-time **Autonomous Racing Non-Linear Model Predictive Controller (MPCC)** designed for Formula Student autonomous vehicles in the **PACSim** simulation environment.

Formulated in curvilinear **Frenet-Serret coordinates**, the controller optimizes vehicle trajectory and dynamics simultaneously: it naturally discovers and executes the **optimal racing line** (widening entries, clipping inside apexes, running wide on exits) within the safe boundary cones, while operating strictly at the limits of tire adhesion with sub-millisecond execution times.

---

## 📑 Table of Contents
- [Key Features](#key-features)
- [Repository Structure](#repository-structure)
- [Build & Run Workflow](#build--run-workflow)
- [Parameters at a Glance](#parameters-at-a-glance)
- [Visualization in Foxglove Studio](#visualization-in-foxglove-studio)
- [Documentation Suite](#documentation-suite)
- [Prerequisites & Dependencies](#prerequisites--dependencies)

---

## Key Features

- **Autonomous Racing Line Emergence (MPCC)**: The controller is not constrained to follow the centerline. Driven by speed maximization ($w_v = 4.0$) and mild lateral centering ($q_{ey} = 0.08$), it naturally cuts apexes and exploits the full width of the cone corridor.
- **Stage-Dependent Cone Corridor Bounds**: Cubic splines fitted to track cones determine the safe lateral corridor $e_y \in [-w_r(s_k) + d_{\mathrm{margin}}, w_l(s_k) - d_{\mathrm{margin}}]$ at each preview stage, guaranteeing 0 cone strikes.
- **Kamm Friction Circle Grip Capping**: Protects against snap oversteer by capping longitudinal drive force during high-lateral-g cornering:

  $$
  a_{\mathrm{lon,kamm}} = \sqrt{\max\left(0.4, (0.92 \mu g)^2 - a_{\mathrm{lat}}^2\right)}
  $$

- **Curvature-Adaptive Speed Envelope & Backward Braking Pass**: Backward integration propagates deceleration from upcoming corners up to 65 m ahead, initiating stable straight-line braking before hairpin turns.
- **Sub-Millisecond Execution (1.32 - 1.43 ms)**: Powered by `acados` Real-Time Iteration (SQP-RTI) with the HPIPM interior-point solver, using $< 15\%$ of the 10.0 ms (100 Hz) control period.
- **Dynamic Launch Governor**: Energetic standing start launch (4.8 m/s²) on straights, smoothly derating on low-speed corner exits.
- **Live Terminal Lap Logger & Exit Trap**: Automatically detects gate crossings and prints live lap time banners and post-run diagnostic scorecards.
- **Single Source of Truth (SSOT)**: All constraints, speeds, margins, and weights are centralized in [`config/mpc_params.yaml`](config/mpc_params.yaml).

---

## Repository Structure

```
etdv_mpc/
├── CMakeLists.txt              # Release-optimized build script with acados linking
├── package.xml                 # ROS 2 package metadata and dependencies
├── config/
│   ├── mpc_params.yaml         # Single Source of Truth (SSOT) parameter file
│   └── velocity_curvature_limits.csv # Empirical vehicle cornering limit table
├── python_model/
│   ├── kinematic_frenet_model.py # Symbolic CasADi model definition in Frenet frame
│   ├── generate_c_code.py      # Automated acados OCP generator (reads YAML directly)
│   ├── test_mpc_solver.py      # Standalone Python solver verification script
│   └── requirements.txt        # Python dependencies (acados_template, casadi, numpy)
├── c_generated_code/           # Auto-generated high-performance C source files
├── include/
│   ├── mpc_pacsim_node.hpp     # Main ROS 2 controller node orchestrator
│   ├── acados_mpc_solver.hpp   # C++ capsule wrapper for acados solver
│   ├── frenet_track.hpp        # Cubic spline and Frenet coordinate transformations
│   ├── speed_governor.hpp      # Speed profile generation and backward braking pass
│   └── utils.hpp               # Foxglove visualizers, metrics, and telemetry logger
├── src/
│   ├── mpc_pacsim_node.cpp     # 100 Hz execution loop, TF lookup, parameter management
│   ├── acados_mpc_solver.cpp   # RTI feedback execution and stage-wise parameter updates
│   ├── frenet_track.cpp        # Continuous curvature spline and Frenet projections
│   ├── speed_governor.cpp      # Speed profile generation and backward braking pass
│   └── utils.cpp               # Marker generation and CSV telemetry implementation
├── launch/
│   └── mpc_pacsimlaunch.py     # Launch file for PACSim with controller
├── docs/
│   ├── THEORY.md               # Detailed mathematical modeling and MPCC derivation
│   └── PARAMETERS.md           # Exhaustive parameter reference & tuning handbook
└── scripts/
    ├── analyze_multitrack.py   # Cross-track diagnostic & publication dashboard generator
    ├── analyze_run.py          # Single-run diagnostic & parameter advisory engine
    ├── densify_centerline.py   # Smooth geometric centerline generator from cones
    ├── multi_track_sentinel.py # Universal multi-track 2-lap referee sentinel
    ├── plot_mpc_telemetry.py   # Comparative telemetry visualizer (PID vs MPC)
    ├── plot_mpc_timing.py      # Solver latency and execution loop time plotter
    └── setup_acados.sh         # Helper installer for acados and Python environment
```

---

## Build & Run Workflow

### Step 1: Generate acados C Solver Code
Before compiling the ROS 2 package, generate the C solver source code from the symbolic CasADi model (which reads configuration parameters directly from `config/mpc_params.yaml`):

```bash
# Ensure acados environment variables are set
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-/opt/acados}"
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:$LD_LIBRARY_PATH"

# Run code generator
cd python_model
python3 generate_c_code.py
cd ..
```
*This produces high-performance C files inside `c_generated_code/`.*

### Step 2: Build the ROS 2 Package with Colcon
From your ROS 2 workspace root:

```bash
colcon build --packages-select etdv_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Step 3: Launch Controller with PACSim
Start simulation and controller:

```bash
# Default track (FSE23_dense_centerline.yaml)
ros2 launch etdv_mpc mpc_pacsimlaunch.py

# Or specify a custom track from pacsim tracks directory
ros2 launch etdv_mpc mpc_pacsimlaunch.py track_name:=FSG21_dense_centerline.yaml
```

### Step 4: Telemetry & Performance Diagnostics (Optional)
In a separate terminal, monitor gate passes and generate post-run performance reports:

```bash
# Run universal multi-track lap sentinel
python3 scripts/multi_track_sentinel.py --track FSG21 --laps 2 --timeout 120

# Generate single-run diagnostic scorecard and telemetry plots
python3 scripts/analyze_run.py --log-dir MPC_logs
```

---

## Parameters at a Glance

All parameters are centralized in [`config/mpc_params.yaml`](config/mpc_params.yaml):

| Parameter | Current Value | Unit | Description |
| :--- | :---: | :--- | :--- |
| `control_rate` | `100.0` | Hz | Outer control loop frequency (10.0 ms budget) |
| `mpc_dt` | `0.05` | s | Horizon discretization step ($N=30 \to 1.5\mathrm{\ s}$ preview) |
| `track_margin` | `0.80` | m | Clearance buffer from cone boundaries (unlocks ±0.70 m corridor) |
| `speed_scale` | `1.00` | - | Empirical curvature limit multiplier (100% grip utilization) |
| `max_straight_speed` | `25.0` | m/s | Top speed permitted on straightaways (90.0 km/h) |
| `max_accel` | `4.8` | m/s² | Maximum longitudinal acceleration in OCP & powertrain |
| `a_brake` | `5.0` | m/s² | Deceleration rate for backward-pass preview braking |
| `standing_launch_accel` | `4.8` | m/s² | Straight-line acceleration from standstill |
| `low_speed_max_accel` | `2.20` | m/s² | Corner-exit low-speed acceleration baseline |
| `corner_exit_steer_derate` | `0.55` | - | Proportional torque attenuation with steering angle |
| `max_accel_slew_rate` | `9.0` | m/s³ | Rapid throttle rise slew rate on straights |
| `understeer_gradient` | `0.0008` | rad/(m/s²) | Dynamic tire slip angle compensation |

👉 *For the complete reference and tuning guidelines, see the [Parameter Configuration & Tuning Guide](docs/PARAMETERS.md).*

---

## Visualization in Foxglove Studio

The controller publishes high-rate visualization markers:
- `/mpc/predicted_path` (`visualization_msgs/Marker`): Cyan trajectory showing the optimal predicted path over the 1.5 s horizon.
- `/mpc/predicted_spheres` (`visualization_msgs/Marker`): Spheres indicating the predicted vehicle poses at each horizon stage.
- `/mpc/reference_path` (`visualization_msgs/Marker`): Continuous orange line of the spline-interpolated centerline.
- `/mpc/reference_spheres` (`visualization_msgs/Marker`): Lookahead reference target points.

---

## Documentation Suite

- 📖 **[Mathematical Theory & OCP Formulation](docs/THEORY.md)**: Mathematical derivation of the Frenet frame equations, emergent racing line proof, soft friction circle slacks, and SQP-RTI solver details.
- ⚙️ **[Parameter Configuration & Tuning Guide](docs/PARAMETERS.md)**: Exhaustive breakdown of every parameter in `mpc_params.yaml`, physical units, and category-by-category tuning explanations.

---

## Prerequisites & Dependencies

- **Ubuntu 22.04 LTS** (or Docker container)
- **ROS 2 Humble Hawksbill**
- **acados** (built with HPIPM and BLASFEO)
- **CasADi** (v3.6+)
- **Eigen3**, **tf2_ros**, **pacsim**
