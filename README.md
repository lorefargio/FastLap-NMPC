# FastLap NMPC: Autonomous Racing Model Predictive Contouring Control in Frenet Frame

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg?logo=python)](https://www.python.org/)
[![acados](https://img.shields.io/badge/acados-SQP--RTI-brightgreen.svg)](https://docs.acados.org/)
[![CasADi](https://img.shields.io/badge/CasADi-3.6+-orange.svg)](https://web.casadi.org/)
![Safety](https://img.shields.io/badge/Cone%20Safety-40.0%2F40%20%280%20Cones%29-brightgreen.svg)

High-performance, real-time **Autonomous Racing Non-Linear Model Predictive Controller (MPCC)** designed for Formula Student autonomous vehicles in the **PACSim** simulation environment.

Formulated in curvilinear **Frenet-Serret coordinates**, the controller optimizes vehicle trajectory and dynamics simultaneously: it naturally discovers and executes the **optimal racing line** (widening entries, clipping inside apexes, running wide on exits) within the safe boundary cones, while operating strictly at the limits of tire adhesion with sub-millisecond execution times.

---

## 📑 Table of Contents
- [Key Features](#key-features)
- [Repository Structure](#repository-structure)
- [Quickstart Guide](#quickstart-guide)
- [Parameters at a Glance](#parameters-at-a-glance)
- [Multi-Track Performance & KPIs](#multi-track-performance--kpis)
- [Official Formula Student Penalty Comparison](#official-formula-student-penalty-comparison)
- [Visualization in Foxglove Studio](#visualization-in-foxglove-studio)
- [Documentation Suite](#documentation-suite)
- [Prerequisites & Dependencies](#prerequisites--dependencies)

---

## Key Features

- **Autonomous Racing Line Emergence (MPCC)**: The controller is not constrained to follow the centerline. Driven by speed maximization ($w_v = 4.0$) and mild lateral centering ($q_{ey} = 0.08$), it naturally cuts apexes and exploits the full width of the cone corridor.
- **Stage-Dependent Cone Corridor Bounds**: Cubic splines fitted to track cones determine the safe lateral corridor $e_y \in [-w_r(s_k) + \text{margin}, w_l(s_k) - \text{margin}]$ at each preview stage, guaranteeing **0 cone strikes**.
- **Kamm Friction Circle Grip Capping**: Protects against snap oversteer by capping longitudinal drive force during high-lateral-g cornering:

  $$
  a_{\text{lon,kamm}} = \sqrt{\max\left(0.4,\, (0.92 \mu g)^2 - a_{\text{lat}}^2\right)}
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
│   ├── PARAMETERS.md           # Exhaustive parameter reference & tuning handbook
│   └── PERFORMANCE.md          # Multi-track evaluation, timing metrics, and KPIs
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

## Quickstart Guide

Inside the Docker development environment:

### 1. Build and Compile
```bash
# Automatically generates C code from python_model and compiles the node
./build_mpc.sh
```

### 2. Run Single Track Test (with Live Lap Times)
```bash
# Default track or specify track_name
./run_mpc.sh track_name:=FSG21_dense_centerline.yaml
```
*When the run finishes or is interrupted with `Ctrl+C`, a comprehensive diagnostic scorecard is printed to the terminal automatically.*

### 3. Run Universal Multi-Track Benchmark
```bash
python3 scripts/multi_track_sentinel.py --laps 2 --timeout 120
```

---

## Parameters at a Glance

All parameters are centralized in [`config/mpc_params.yaml`](config/mpc_params.yaml):

| Parameter | Current Value | Unit | Description |
| :--- | :---: | :--- | :--- |
| `control_rate` | `100.0` | Hz | Outer control loop frequency (10.0 ms budget) |
| `mpc_dt` | `0.05` | s | Horizon discretization step ($N=30 \to 1.5\text{ s}$ preview) |
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

## Multi-Track Performance & KPIs

Evaluated in a standardized **2-Lap Battery** across three official international circuits:

| Metric | FSE23 (Spain) | FSG21 (Germany) | FSI24 (Italy) | Overall Status |
| :--- | :---: | :---: | :---: | :---: |
| **Track Length** | ~298.0 m | ~380.0 m | ~390.0 m | Cross-Track Suite |
| **Lap 1: Standing Start** | **21.720 s** | **18.451 s** | **30.210 s** | Rapid Launch |
| **Lap 2: Flying Lap** | **20.830 s** | **17.469 s** | **29.041 s** | **Record Lap Times** |
| **Total 2-Lap Time** | **42.550 s** | **35.920 s** | **59.251 s** | Consistent |
| **Flying Top Speed** | **80.2 km/h** | **69.0 km/h** | **87.1 km/h** | Unthrottled Straights |
| **Flying Avg Speed** | **41.4 km/h** | **45.2 km/h** | **46.8 km/h** | Optimal Pace |
| **Physical Cone Strikes (DOO)** | **0** | **0** | **0** | **40.0 / 40 Safety Score** |
| **Minimum Cone Clearance** | **0.750 m** | **0.756 m** | **0.757 m** | **Symmetric Precision** |
| **Mean acados Solve Time** | **1.38 ms** | **1.32 ms** | **1.43 ms** | **< 15% of 10ms Budget** |
| **P99 Solver Latency** | **4.40 ms** | **3.82 ms** | **3.92 ms** | Real-Time Deterministic |

---

## Official Formula Student Penalty Comparison

In Formula Student regulations, each cone knocked down (**DOO - Down or Out**) incurs a **+2.0 s penalty**:

- **FSE23**: from 53.19 s (crash with 14 cones in unconstrained MPCC) $\to$ **20.830 s clean** (**-32.36 s gain**).
- **FSG21**: from 32.87 s (16.87s + 16s for 8 cones) $\to$ **17.469 s clean** (**-15.40 s gain**).
- **FSI24**: from 29.72 s (27.72s + 2s for 1 cono) $\to$ **29.041 s clean** (**-0.68 s gain**).

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
- 📊 **[Track Performance & Evaluation](docs/PERFORMANCE.md)**: Official multi-track benchmark results across FSE23, FSG21, and FSI24, solver timing distributions, and telemetry tools.

---

## Prerequisites & Dependencies

- **Ubuntu 22.04 LTS** (or Docker container)
- **ROS 2 Humble Hawksbill**
- **acados** (built with HPIPM and BLASFEO)
- **CasADi** (v3.6+)
- **Eigen3**, **tf2_ros**, **pacsim**
