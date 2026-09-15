# ETDV MPC (Minimum Lap Time NMPC in Frenet Frame with acados)

Real-Time Non-Linear Model Predictive Controller (NMPC) for the PACSim autonomous driving simulator using ROS 2 Humble.

## Key Features
- **Kinematic Single-Track (Bicycle) Model**: Parameterized in Frenet-Serret coordinates ($s, e_y, e_\psi, v, \delta$) along the track centerline.
- **Minimum Lap Time Optimization**: Formulated via progress maximization along the reference path, allowing apex cutting while strictly respecting track width constraints ($e_y \in [-w_r, w_l]$) and tire grip limits (friction circle $a_{lon}^2 + a_{lat}^2 \le (\mu g)^2$).
- **Sub-Millisecond Execution**: Generated using `acados` Real-Time Iteration (RTI) Gauss-Newton SQP solver with HPIPM.
- **PACSim & `etdv_pid` Compatibility**: Seamless plug-and-play actuation (steering ratio scaling and RWD/4-wheel braking torque allocation) and trajectory acquisition (`/pacsim/track/centerline_smoothed_front`).

---

## Quickstart Inside Container

### 1. Environment & acados Setup
Run the setup script inside your container to build acados and configure your Python virtual environment:
```bash
cd /ros2_ws/etdv_mpc
chmod +x scripts/setup_acados.sh
./scripts/setup_acados.sh /opt/acados ~/.venv_mpc
```
Ensure your environment variables are sourced:
```bash
export ACADOS_SOURCE_DIR="/opt/acados"
export LD_LIBRARY_PATH="/opt/acados/lib:$LD_LIBRARY_PATH"
export PATH="/opt/acados/bin:$PATH"
source ~/.venv_mpc/bin/activate
```

### 2. Generate the acados C Solver Code
```bash
cd /ros2_ws/etdv_mpc/python_model
python3 generate_c_code.py
```
This generates the optimized C code in `/ros2_ws/etdv_mpc/c_generated_code/`.

Verify offline:
```bash
python3 test_mpc_solver.py
```

### 3. Build the ROS 2 Package
```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select etdv_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4. Launch PACSim with ETDV NMPC
```bash
ros2 launch etdv_mpc mpc_pacsimlaunch_firstLap.py
```

### 5. Benchmark Telemetry vs PID
```bash
python3 /ros2_ws/etdv_mpc/scripts/plot_mpc_telemetry.py \
    --pid-dir /ros2_ws/PID_logs \
    --mpc-dir /ros2_ws/MPC_logs \
    --output /ros2_ws/comparison_report.png
```
