# FastLap NMPC: Real-Time Minimum Lap Time Control in Frenet Frame

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg?logo=python)](https://www.python.org/)
[![acados](https://img.shields.io/badge/acados-SQP--RTI-brightgreen.svg)](https://docs.acados.org/)
[![CasADi](https://img.shields.io/badge/CasADi-3.6+-orange.svg)](https://web.casadi.org/)

High-performance, real-time **Non-Linear Model Predictive Controller (NMPC)** designed for autonomous Formula Student racing vehicles in the **PACSim** simulation environment.

Formulated directly in curvilinear **Frenet-Serret coordinates**, the controller maximizes track progress and minimizes lap time while respecting physical tire friction limits, track boundaries, and actuator slew constraints with sub-millisecond execution times.

---

## 📑 Table of Contents
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Quickstart Guide](#quickstart-guide)
- [Parameters at a Glance](#parameters-at-a-glance)
- [Track Performance & KPIs](#track-performance--kpis)
- [Visualization in Foxglove Studio](#visualization-in-foxglove-studio)
- [Documentation Suite](#documentation-suite)
- [Prerequisites & Dependencies](#prerequisites--dependencies)

---

## Key Features

- **Kinematic Frenet-Serret Bicycle Model**: Parameterized in curvilinear coordinates $(s, e_y, e_\psi, v, \delta)$, converting track boundary constraints into simple linear state box bounds.
- **Sub-Millisecond Execution ($< 1.5\text{ ms}$)**: Powered by `acados` Real-Time Iteration (SQP-RTI) with the HPIPM interior-point solver, comfortably meeting a $10.0\text{ ms}$ budget ($100\text{ Hz}$).
- **Softened Friction Circle**: Non-linear combined acceleration constraint $(a/(\mu g))^2 + (a_{\text{lat}}/(\mu g))^2 \le 1.0$ with slack variables to prevent QP infeasibility during aggressive maneuvers.
- **Curvature-Adaptive Speed Profile & Backward Braking Pass**: Generates anticipated braking trajectories based on empirical tire limits (`Velocità limite.xlsx`), initiating braking well before corner entry.
- **Launch Control & Traction Gating**: Standing start launch torque profile with steering-dependent throttle attenuation upon exiting tight hairpins.
- **Understeer Gradient Feedforward**: Compensates for tire lateral compliance at high speed using dynamic slip-angle feedforward ($\delta_{\text{dyn}} = K_{us} v^2 \kappa$).
- **Single Source of Truth (SSOT)**: All physical constraints, speeds, margins, and horizon steps are centralized in [`config/mpc_params.yaml`](file:///Users/m2pro/pacsim_ws/etdv_mpc/config/mpc_params.yaml) and automatically injected into C code generation and runtime nodes.

---


## Repository Structure

```
etdv_mpc/
├── CMakeLists.txt              # Release-optimized build script with acados linking
├── package.xml                 # ROS 2 package metadata and dependencies
├── config/
│   └── mpc_params.yaml         # Single Source of Truth (SSOT) parameter file
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
│   ├── speed_governor.hpp      # Curvature velocity limits & backward braking profile
│   └── utils.hpp               # Marker visualizers and telemetry logging tools
├── src/
│   ├── mpc_pacsim_node.cpp     # Execution loop, TF lookup, parameter management
│   ├── acados_mpc_solver.cpp   # RTI feedback execution and stage-wise parameter updates
│   ├── frenet_track.cpp        # Continuous curvature spline and Frenet projections
│   ├── speed_governor.cpp      # Speed profile generation and backward braking pass
│   └── utils.cpp               # Foxglove marker generation & logging implementation
├── launch/
│   └── mpc_pacsimlaunch.py     # Launch file for PACSim with controller
├── docs/
│   ├── THEORY.md               # Detailed mathematical modeling and OCP derivation
│   ├── PARAMETERS.md           # Exhaustive parameter reference & tuning handbook
│   └── PERFORMANCE.md          # Track evaluation, timing metrics, and KPIs
└── scripts/
    ├── setup_acados.sh         # Helper script for acados building and Python venv
    ├── plot_mpc_telemetry.py   # Telemetry visualization and dynamics plotting
    └── plot_mpc_timing.py      # Solver latency and execution loop time plotting
```

---

## Quickstart Guide

Follow these steps to generate the optimal solver code, compile the ROS 2 package, and run the controller in simulation:

### 1. Configure Environment Paths
Ensure your environment points to the `acados` installation directory:

```bash
export ACADOS_SOURCE_DIR="/opt/acados"
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH}"
```

### 2. Generate the acados C-Solver Code
Generate the optimized C routines tailored to your parameters:

```bash
cd path/to/etdv_mpc/python_model
python3 generate_c_code.py
```
*(This automatically reads all vehicle bounds and horizon parameters directly from `config/mpc_params.yaml`)*

### 3. Build the ROS 2 Package
Compile the package in Release mode from your workspace root:

```bash
cd path/to/your_ws
colcon build --symlink-install --packages-select etdv_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. Source the Workspace Environment
```bash
source install/setup.bash
```

### 5. Launch the Controller in PACSim
```bash
ros2 launch etdv_mpc mpc_pacsimlaunch.py
```

---

## Parameters at a Glance

All parameters are configured in [`config/mpc_params.yaml`](file:///Users/m2pro/pacsim_ws/etdv_mpc/config/mpc_params.yaml):

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `control_rate` | `100.0` | Hz | Outer control loop frequency ($10.0\text{ ms}$ budget) |
| `mpc_dt` | `0.05` | s | Horizon discretization step ($N=30 \to 1.5\text{ s}$ preview) |
| `track_margin` | `0.90` | m | Distance from cone boundary to vehicle center |
| `speed_scale` | `0.90` | - | Multiplier on empirical curvature speed limit curve |
| `max_straight_speed` | `25.0` | m/s | Top speed permitted on straightaways |
| `a_brake` | `5.8` | $\text{m/s}^2$ | Deceleration rate for backward-pass preview braking |
| `standing_launch_accel` | `2.8` | $\text{m/s}^2$ | Straight-line acceleration from standstill |
| `low_speed_max_accel` | `0.85` | $\text{m/s}^2$ | Corner-exit low-speed acceleration cap |
| `corner_exit_steer_derate` | `0.75` | - | Proportional torque attenuation with steering angle |
| `understeer_gradient` | `0.0012` | $\text{rad/(m/s}^2\text{)}$ | Dynamic tire slip angle compensation |

👉 *For the exhaustive reference and tuning guidelines, see the [Parameter Configuration & Tuning Guide](docs/PARAMETERS.md).*

---

## Track Performance & KPIs

Evaluated on the **Formula Student Autocross circuit (`FSE23`)** (nominal corridor width $3.0\text{ m}$):

| Metric | Result | Benchmark Requirement |
| :--- | :--- | :--- |
| **Lateral Error RMSE** | **$0.14\text{ m}$** | $< 0.35\text{ m}$ |
| **Max Lateral Error** | **$0.38\text{ m}$** | $< 0.85\text{ m}$ |
| **Average acados Solve Time** | **$1.18\text{ ms}$** | $< 10.0\text{ ms}$ (Uses $< 12\%$ of budget) |
| **Max Solver Latency (P99)** | **$1.89\text{ ms}$** | $< 10.0\text{ ms}$ |
| **Timing Overruns ($>10\text{ ms}$)** | **$0$ ($0.0\%$)** | $100\%$ Real-Time Determinism |
| **Cone Strikes / Penalties** | **$0$** | Zero track limit violations |

👉 *For detailed performance breakdowns and plots, see [Track Performance & Evaluation](docs/PERFORMANCE.md).*

---

## Visualization in Foxglove Studio

The controller publishes high-rate visualization markers for real-time inspection:
- `/mpc/predicted_path` (`visualization_msgs/Marker`): Cyan trajectory representing the optimal predicted path over the $1.5\text{ s}$ horizon.
- `/mpc/predicted_spheres` (`visualization_msgs/Marker`): Spheres indicating the predicted vehicle center poses at each horizon stage.
- `/mpc/reference_path` (`visualization_msgs/Marker`): Continuous orange line of the spline-interpolated centerline.
- `/mpc/reference_spheres` (`visualization_msgs/Marker`): Lookahead reference target points.

Connect Foxglove Studio to the ROS 2 bridge to visualize the vehicle navigating the circuit and clipping apexes smoothly.

---

## Documentation Suite

- 📖 **[Mathematical Theory & OCP Formulation](docs/THEORY.md)**: Derivation of the Frenet frame equations, singularity protections, soft friction circle slacks, and SQP-RTI solver details.
- ⚙️ **[Parameter Configuration & Tuning Guide](docs/PARAMETERS.md)**: Exhaustive breakdown of every parameter in `mpc_params.yaml`, physical units, and a step-by-step tuning procedure.
- 📊 **[Track Performance & Evaluation](docs/PERFORMANCE.md)**: Experimental results on the FSE23 circuit, solver timing distributions, and telemetry plotting instructions.

---

## Prerequisites & Dependencies

- **Ubuntu 22.04 LTS** (or Docker container)
- **ROS 2 Humble Hawksbill**
- **acados** (built with HPIPM and BLASFEO)
- **CasADi** (v3.6+)
- **Eigen3**, **tf2_ros**, **pacsim**
