# Mathematical Modeling and Optimal Control Problem (OCP) Formulation

This document details the theoretical derivation of the Non-Linear Model Predictive Controller (NMPC) implemented in FastLap NMPC. The controller is designed for high-performance trajectory tracking and minimum-lap-time optimization of an autonomous Formula Student vehicle in the PACSim simulation environment.

---

## 1. Curvilinear Frenet-Serret Coordinate Frame

Standard vehicle models parameterized in global Cartesian coordinates $(X, Y, \Psi)$ require complex, non-convex representations of track boundaries. By reformulating vehicle kinematics along a reference centerline in curvilinear Frenet coordinates, track boundaries translate directly into linear state box constraints:

$$\mathbf{x}_{\text{Frenet}} = \begin{bmatrix} s \\ e_y \\ e_\psi \\ v \\ \delta \end{bmatrix}$$

where:
- $s \in [0, L_{\text{track}}]$: Curvilinear arc length along the track centerline $[m]$.
- $e_y$: Lateral deviation of the vehicle center of gravity (CG) from the reference curve $[m]$ (positive to the left, negative to the right).
- $e_\psi = \psi - \psi_{\text{ref}}(s)$: Heading error relative to the tangent of the reference path at arc length $s$ $[\text{rad}]$.
- $v$: Longitudinal velocity along the vehicle body x-axis $[\text{m/s}]$.
- $\delta$: Wheel steering angle $[\text{rad}]$.

```
                  Reference Path (Tangent T)
                          ^
                         /
                        /
                       /  psi_ref(s)
                      +--------------------> Global X
                     /
                    /
                   /   e_y (normal offset)
      Vehicle CG  * <---------------------+ Reference Point at arc length s
                  | \
                  |  \  psi (Vehicle Heading)
                  |   \
                  v    \
```

---

## 2. Kinematic Single-Track (Bicycle) Model

For control frequencies around $100\text{ Hz}$ and speeds up to $\approx 25\text{ m/s}$, the kinematic single-track model accurately captures cornering geometry while maintaining real-time QP solvability.

### 2.1 Side-Slip Angle at Vehicle CG

The side-slip angle $\beta$ at the vehicle center of gravity is computed geometrically from the front and rear axle distances ($l_f, l_r$) and the wheelbase $L = l_f + l_r$:

$$\beta(\delta) = \arctan\left(\frac{l_r}{L} \tan \delta\right)$$

In PACSim, vehicle parameters from `vehicleModel.yaml` are:
- $l_f = 0.69\text{ m}$
- $l_r = 0.84\text{ m}$
- $L = 1.53\text{ m}$
- $g = 9.81\text{ m/s}^2$

### 2.2 Global Yaw Rate

The global yaw rate of the vehicle chassis is:

$$\dot{\psi}_{\text{veh}} = \frac{v \cos \beta(\delta)}{L} \tan \delta$$

### 2.3 Curvilinear State Derivatives

Taking time derivatives with respect to the moving Frenet-Serret frame:

1. **Progress along centerline ($\dot{s}$)**:
   $$\dot{s} = \frac{v \cos(e_\psi + \beta)}{1 - e_y \kappa(s)}$$
   *Singularity Protection*: If the vehicle approaches the center of curvature ($e_y \to 1/\kappa$), the denominator approaches zero. To guarantee numerical stability in acados, the denominator is bounded:
   $$\operatorname{denom} = \max\left(0.05,\, 1 - e_y \kappa(s)\right)$$

2. **Lateral error evolution ($\dot{e}_y$)**:
   $$\dot{e}_y = v \sin(e_\psi + \beta)$$

3. **Heading error rate ($\dot{e}_\psi$)**:
   $$\dot{e}_\psi = \dot{\psi}_{\text{veh}} - \kappa(s) \dot{s} = \frac{v \cos \beta}{L} \tan \delta - \kappa(s) \frac{v \cos(e_\psi + \beta)}{\operatorname{denom}}$$

4. **Longitudinal velocity ($\dot{v}$)**:
   $$\dot{v} = a$$
   where $a$ is the net longitudinal acceleration command $[\text{m/s}^2]$.

5. **Wheel steering angle ($\dot{\delta}$)**:
   $$\dot{\delta} = v_\delta$$
   where $v_\delta$ is the steering angular velocity command $[\text{rad/s}]$.

The explicit ordinary differential equation (ODE) vector field is:

$$\dot{\mathbf{x}} = \mathbf{f}_{\text{expl}}(\mathbf{x}, \mathbf{u}, \mathbf{p}) = \begin{bmatrix}
\frac{v \cos(e_\psi + \beta)}{\max(0.05, 1 - e_y \kappa)} \\
v \sin(e_\psi + \beta) \\
\frac{v \cos\beta}{L} \tan\delta - \kappa \dot{s} \\
a \\
v_\delta
\end{bmatrix}$$

with control inputs $\mathbf{u} = [a, v_\delta]^T$ and runtime parameters $\mathbf{p} = [\kappa, w_l, w_r, \mu]^T$.

---

## 3. Non-Linear Friction Circle Constraint

To enforce tire grip limits, the total vehicle acceleration is bounded by the available tire-road friction $\mu$:

$$a_{\text{lon}}^2 + a_{\text{lat}}^2 \le (\mu g)^2$$

where:
- $a_{\text{lon}} = a$ (commanded longitudinal acceleration)
- $a_{\text{lat}} = v \cdot \dot{\psi}_{\text{veh}} = \frac{v^2 \cos\beta}{L} \tan\delta$ (lateral cornering acceleration)

The normalized non-linear constraint is:

$$h(\mathbf{x}, \mathbf{u}, \mathbf{p}) = \frac{a^2 + a_{\text{lat}}^2}{(\mu g)^2} \le 1.0$$

### Slack Variable Formulation
In extreme transient maneuvers (e.g. sharp entry into hairpins at high speed), a rigid constraint can cause QP infeasibility in acados, forcing emergency stops. We soften this constraint with a lower/upper slack variable $s_{\text{friction}} \ge 0$:

$$h(\mathbf{x}, \mathbf{u}, \mathbf{p}) \le 1.0 + s_{\text{friction}}$$

with high linear and quadratic penalties ($z_u = 100.0$, $Z_u = 500.0$) in the OCP cost, guaranteeing feasibility at all operating points while keeping physical grip violations negligible.

---

## 4. Optimal Control Problem (OCP) Formulation

The discrete-time optimal control problem over a prediction horizon of $N = 30$ stages with sampling time $\Delta t = 0.05\text{ s}$ ($T_f = 1.5\text{ s}$) is formulated as:

$$\min_{\mathbf{x}_{0:N}, \mathbf{u}_{0:N-1}, \mathbf{s}_{0:N-1}} \sum_{k=0}^{N-1} \left( \frac{1}{2} \|\mathbf{y}_k - \mathbf{y}_{\text{ref},k}\|_{\mathbf{W}}^2 + z_l^T s_{l,k} + z_u^T s_{u,k} + \frac{1}{2} s_{l,k}^T Z_l s_{l,k} + \frac{1}{2} s_{u,k}^T Z_u s_{u,k} \right) + \frac{1}{2} \|\mathbf{y}_N - \mathbf{y}_{\text{ref},N}\|_{\mathbf{W}_e}^2$$

### 4.1 Residual Vectors
- **Stage Residual**:
  $$\mathbf{y}_k = [v_k,\, e_{y,k},\, e_{\psi,k},\, \delta_k,\, a_k,\, v_{\delta,k}]^T$$
  $$\mathbf{y}_{\text{ref},k} = [v_{\text{target},k},\, 0,\, 0,\, 0,\, 0,\, 0]^T$$

- **Terminal Residual**:
  $$\mathbf{y}_N = [v_N,\, e_{y,N},\, e_{\psi,N},\, \delta_N]^T$$
  $$\mathbf{y}_{\text{ref},N} = [v_{\text{target},N},\, 0,\, 0,\, 0]^T$$

### 4.2 Weighting Matrices
The weighting matrices balance fast progress with smooth actuation:

$$\mathbf{W} = \operatorname{diag}\begin{bmatrix}
w_v & = & 2.5 & \text{(Speed tracking)} \\
q_{e_y} & = & 15.0 & \text{(Lateral tracking accuracy)} \\
q_{e_\psi} & = & 18.0 & \text{(Heading alignment to track tangent)} \\
r_\delta & = & 0.30 & \text{(Steering angle regularization)} \\
r_a & = & 0.40 & \text{(Longitudinal acceleration smoothness)} \\
r_{v_\delta} & = & 2.50 & \text{(Steering rate damping - eliminates steering oscillations)}
\end{bmatrix}$$

Terminal weights are scaled by $1.5\times$ ($\mathbf{W}_e = \operatorname{diag}[3.75, 22.5, 27.0, 0.30]$) to enforce terminal stability.

---

## 5. Box Constraints & Bounds

| Variable | Lower Bound | Upper Bound | Description |
| :--- | :--- | :--- | :--- |
| **$e_y$** | $-w_{r} + \Delta_{\text{margin}}$ | $w_{l} - \Delta_{\text{margin}}$ | Track boundaries (soft slack enabled) |
| **$v$** | $0.0\text{ m/s}$ | $\max(35.0, 1.3 \cdot v_{\text{straight}})\text{ m/s}$ | Prevents speed-related infeasibility |
| **$\delta$** | $-0.52\text{ rad}$ ($-30^\circ$) | $0.52\text{ rad}$ ($+30^\circ$) | Maximum mechanical steering limit |
| **$a$** | $-8.0\text{ m/s}^2$ | $3.5\text{ m/s}^2$ | Powertrain and braking capacity |
| **$v_\delta$** | $-1.5\text{ rad/s}$ | $1.5\text{ rad/s}$ | Steering actuator velocity limit |

---

## 6. Real-Time Iteration (SQP-RTI) Scheme

To execute within the strict $10.0\text{ ms}$ control budget of ROS 2 at $100\text{ Hz}$, the problem is solved using the **acados SQP-RTI (Real-Time Iteration)** scheme:

1. **Preparation Phase (inter-sample)**:
   - Integrates the continuous system dynamics using 4th-order Runge-Kutta (ERK4).
   - Linearizes dynamics and constraints around the previous solution trajectory.
   - Condenses the QP problem using Partial Condensing ($N_{\text{cond}} = 5$).
2. **Feedback Phase (immediate on state arrival)**:
   - Sets the measured initial state $\mathbf{x}_0$.
   - Performs a single QP solve using the high-performance interior point solver **HPIPM** (exploiting BLASFEO linear algebra routines).
   - Extracts the optimal actuation $\mathbf{u}_0^* = [a_0^*, v_{\delta,0}^*]^T$ and updates the system in $< 1.5\text{ ms}$.
