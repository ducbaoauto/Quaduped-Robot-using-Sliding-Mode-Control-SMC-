# PROJECT SUMMARY — SMC Trajectory Tracking on Unitree GO1
> Auto-generated from source code. All equations and values copied exactly from implementation.

---

## 1. SYSTEM ARCHITECTURE

### ROS2 Node Graph

```
┌─────────────────────────────────┐
│  Gazebo Simulation              │
│  (go1_gazebo / unitree_guide2)  │
│                                 │
│  Publishes:  /odom              │
│  Subscribes: /cmd_vel           │
└───────────┬─────────────────────┘
            │ /odom (Odometry)
            ▼
┌───────────────────────────────────────────────────┐
│  SMC Controller Node (one of):                    │
│    smc_tracking_node   (smc_tracking.py)          │
│    smc_square_node     (smc_square_node.py)       │
│    smc_figure8_node    (smc_figure8_node.py)      │
│    smc_rounded_square_node                        │
│    smc_superellipse_node                          │
│                                                   │
│  Subscribes: /odom                                │
│  Publishes:  /cmd_vel, /target_pose               │
└───────┬────────────────┬──────────────────────────┘
        │ /cmd_vel       │ /target_pose (PoseStamped)
        │ (Twist)        │
        ▼                ▼
┌───────────┐   ┌────────────────────┐   ┌──────────────────┐
│  Robot    │   │  visualizer_node   │   │  data_logger_node│
│  (plant)  │   │  (visualizer_node) │   │                  │
│           │   │  Sub: /odom        │   │  Sub: /odom      │
│           │   │       /target_pose │   │       /target_pose│
│           │   │  (matplotlib plot) │   │       /cmd_vel   │
└───────────┘   └────────────────────┘   │       /joint_states│
                                         │  Writes: smc_data.csv│
                                         └──────────────────┘
```

### Complete Topic Table

| Topic | Message Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Gazebo / unitree_guide2 | All SMC nodes, visualizer_node, data_logger_node |
| `/cmd_vel` | `geometry_msgs/Twist` | SMC controller node | Gazebo / unitree_guide2, data_logger_node |
| `/target_pose` | `geometry_msgs/PoseStamped` | SMC controller node | visualizer_node, data_logger_node |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo | data_logger_node (subscribed, not currently logged) |

### Control Loop Frequencies

| Node | Frequency | `dt` |
|---|---|---|
| `smc_tracking_node` | 20 Hz | 0.05 s |
| `smc_square_node` | 20 Hz | 0.05 s |
| `smc_figure8_node` | 20 Hz | 0.05 s |
| `smc_rounded_square_node` | 20 Hz | 0.05 s |
| `smc_superellipse_node` | 20 Hz | 0.05 s |
| `data_logger_node` | 50 Hz | 0.02 s |
| `visualizer_node` | ~10 Hz | 100 ms (matplotlib FuncAnimation) |

---

## 2. CONTROLLER ALGORITHM — SMC

### State Vector

The controller operates on a 3-DOF planar state:

```
state = [x, y, yaw]
```

read from `/odom` via `odom_callback()`.

### Error Definition (Global Frame)

```
ex    = xd - x
ey    = yd - y
e_yaw = atan2(sin(yaw_d - yaw), cos(yaw_d - yaw))
```

`e_yaw` is wrapped to `[-π, π]` via the `atan2(sin·, cos·)` identity (see Section 5).

### Body Frame Transformation

Global-frame errors are rotated into the robot body frame before the control law is applied:

```
ex_b =  cos(yaw) * ex + sin(yaw) * ey
ey_b = -sin(yaw) * ex + cos(yaw) * ey
```

This is a standard 2D rotation matrix `R(-yaw)` applied to the error vector `[ex, ey]^T`.

### Sliding Surface

The implicit sliding surface in this implementation is the body-frame position error itself:

```
s_x = ex_b
s_y = ey_b
s_ψ = e_yaw
```

The control law drives `s → 0`, which implies the tracking errors converge.

### Control Law — Smooth SMC with Boundary Layer

```
v_cmd_x = k_v * tanh(ex_b  / phi)
v_cmd_y = k_v * tanh(ey_b  / phi)
w_cmd_z = k_w * tanh(e_yaw / phi)
```

**Gain values (identical across all SMC nodes):**

| Parameter | Value | Role |
|---|---|---|
| `k_v` | `0.5` | Maximum linear velocity command magnitude (m/s) |
| `k_w` | `1.5` | Maximum angular velocity command magnitude (rad/s) |
| `phi` | `0.5` | Boundary layer thickness — controls transition between linear and saturated regime |

### Why `tanh()` Instead of `sign()`

The classical SMC control law uses `u = K * sign(s)`, which switches discontinuously and
causes **chattering** — high-frequency oscillations in the actuator output.

The `tanh` replacement:

```
u = K * tanh(s / phi)
```

creates a **boundary layer** of width `phi` around `s = 0`:
- For `|s| >> phi`: `tanh(s/phi) ≈ ±1` → saturated, full-gain SMC behaviour
- For `|s| << phi`: `tanh(s/phi) ≈ s/phi` → linear proportional control
- The transition is C∞ smooth → no chattering, at the cost of a small steady-state error inside the boundary layer

### Published Control Output

```
Twist.linear.x  = v_cmd_x   [m/s]
Twist.linear.y  = v_cmd_y   [m/s]   ← holonomic (GO1 can sidestep)
Twist.angular.z = w_cmd_z   [rad/s]
```

---

## 3. TRAJECTORY GENERATION — All Trajectories

### 3.1 Circular Trajectory (`smc_tracking_node`)

**Source:** `smc_tracking.py` → `get_desired_trajectory(t)`

**Position:**
```
R     = 3.0       # radius (m)
omega = 0.06667   # rad/s  (~94.25 s per lap)

xd(t) = R * cos(omega * t)
yd(t) = R * sin(omega * t)
```

**Heading (tangent to circle):**
```
dx    = -R * omega * sin(omega * t)
dy    =  R * omega * cos(omega * t)
yaw_d = atan2(dy, dx)
```

**Parameters:**

| Symbol | Value | Unit |
|---|---|---|
| R | 3.0 | m |
| ω | 0.06667 | rad/s |
| T | 2π/ω ≈ 94.25 | s |

**Continuity:** C∞ — trigonometric, infinitely differentiable.

---

### 3.2 Figure-8 / Lemniscate of Bernoulli (`smc_figure8_node`)

**Source:** `smc_figure8_node.py` → `get_desired_trajectory(t)`

**Position:**
```
A     = 3.0       # amplitude (m) — half-width of each lobe
OMEGA = 0.06667   # rad/s (same as circular node)

xd(t) = A * sin(OMEGA * t)
yd(t) = (A / 2) * sin(2 * OMEGA * t)
       = A * sin(OMEGA*t) * cos(OMEGA*t)
```

**Heading (analytical tangent):**
```
dxd/dt = A * OMEGA * cos(OMEGA * t)
dyd/dt = A * OMEGA * cos(2 * OMEGA * t)
yaw_d  = atan2(dyd/dt, dxd/dt)
```

**Parameters:**

| Symbol | Value | Unit |
|---|---|---|
| A | 3.0 | m |
| ω | 0.06667 | rad/s |
| T | 2π/ω ≈ 94.25 | s |

**Shape:** Lemniscate of Bernoulli — two symmetric lobes crossing at origin.
At `t=0`: robot is at origin `(0, 0)`. Lobe extents: x ∈ `[-3, 3]`, y ∈ `[-1.5, 1.5]`.

**Continuity:** C∞ everywhere. Note: `yaw_d` has a discontinuity at the figure-8 crossover
point (`t = 0, T/2, T`) because the trajectory reverses curvature sign — `atan2` wraps
but the controller normalises `e_yaw` so this is handled correctly.

---

### 3.3 Rounded Square / Lamé Curve — n=6 (`smc_rounded_square_node`)

**Source:** `smc_rounded_square_node.py` → `get_desired_trajectory(t)`

**Parameters:**
```
L     = 3.0             # half-side length (m) — corners at ±3.0
T     = 60.0            # lap period (s)
OMEGA = 2*pi / T        # = 0.10472 rad/s
N     = 6.0             # Lamé sharpness exponent
```

**Helper:**
```python
def signed_pow(base, exp):
    return copysign(|base|^exp, base)
```

**Position:**
```
xd(t) = L * signed_pow(cos(OMEGA * t), 2/N)
       = L * sign(cos θ) * |cos θ|^(1/3)    where θ = OMEGA * t

yd(t) = L * signed_pow(sin(OMEGA * t), 2/N)
       = L * sign(sin θ) * |sin θ|^(1/3)
```

**Heading (analytical tangent, with edge-case guard):**
```
exp_p = 2/N = 1/3
exp_d = exp_p - 1 = -2/3

dxd/dt = L * exp_p * |cos θ|^exp_d * (-OMEGA * sin θ)    if |cos θ| >= 1e-6, else 0
dyd/dt = L * exp_p * |sin θ|^exp_d * ( OMEGA * cos θ)    if |sin θ| >= 1e-6, else 0

yaw_d  = atan2(dyd/dt, dxd/dt)
         (falls back to _last_yaw_d if both derivatives are zero)
```

**Continuity:** C0 and C1 almost everywhere. The derivative `exp_d = -2/3` produces
an integrable singularity at the 4 axis-crossing points (every quarter-lap), handled
by the `|cos θ| < 1e-6` guard. The path itself (position) is C∞.

---

### 3.4 Square — Lamé Curve, n=20 (`smc_square_node`)

Same formulas as 3.3 with:
```
L = 3.0, N = 20.0, T = 60.0
exp_p = 2/20 = 0.1,  exp_d = -0.9
```

With `N=20` the curve is visually indistinguishable from a perfect square. Corners
reach ≈ `(±2.998, ±2.998)` (within 0.1% of `L`).

---

### 3.5 Superellipse — n=8 (`smc_superellipse_node`)

Same formulas, different parameters:
```
L = 1.5, N = 8.0, T_LAP = 60.0
OMEGA = 2*pi / T_LAP
exp_p = 2/8 = 0.25,  exp_d = -0.75
```

`signed_pow` is an instance method `self.signed_pow(base, exp)`.
Fallback variable is `self.prev_yaw_d` (initialised to 0.0).

| Node | L (m) | N | Shape |
|---|---|---|---|
| `smc_rounded_square_node` | 3.0 | 6 | Gently rounded square |
| `smc_superellipse_node` | 1.5 | 8 | Moderately rounded square |
| `smc_square_node` | 3.0 | 20 | Near-perfect square |

---

## 4. PLANT MODEL

### Robot
**Unitree GO1** — quadruped legged robot capable of holonomic planar motion
(can translate in x, y and rotate about z simultaneously).

### Software Interface
The controller interfaces to the robot via the `unitree_guide2` package (`junior_ctrl`):
- Press `2` inside `junior_ctrl` → stand mode
- Press `5` → `move_base` mode (begins accepting `/cmd_vel`)

### Plant Input
```
geometry_msgs/Twist:
  linear.x   [m/s]    — forward / backward velocity (body frame)
  linear.y   [m/s]    — lateral velocity (body frame, holonomic)
  angular.z  [rad/s]  — yaw rate
```

### Plant Output
```
nav_msgs/Odometry:
  pose.pose.position.x     [m]    — x position in odom frame
  pose.pose.position.y     [m]    — y position in odom frame
  pose.pose.orientation    [quat] — converted to yaw via euler_from_quaternion
```

### State Variables
```
[x, y, yaw] ∈ ℝ × ℝ × [-π, π]
```

### Implicit Kinematic Model

The controller assumes a **velocity-controlled holonomic planar model**:

```
ẋ   = v_x * cos(yaw) - v_y * sin(yaw)
ẏ   = v_x * sin(yaw) + v_y * cos(yaw)
ψ̇   = ω_z
```

where `v_x`, `v_y`, `ω_z` are the commanded Twist values and are treated as
instantaneously achieved (no dynamics lag modelled). The GO1 leg controller
internally handles gait and stabilisation.

---

## 5. ERROR DEFINITION AND DYNAMICS

### Global Frame Errors

```python
ex    = xd - self.x       # x position error (m)
ey    = yd - self.y       # y position error (m)
e_yaw = atan2(sin(yaw_d - self.yaw), cos(yaw_d - self.yaw))   # heading error (rad)
```

### Why `atan2(sin(·), cos(·))` for Angle Wrapping

Raw subtraction `yaw_d - yaw` can produce values outside `[-π, π]`. For example,
if `yaw_d = π - 0.1` and `yaw = -π + 0.1`, naive subtraction gives `≈ 2π - 0.2 ≈ 6.08`,
but the true shortest angular error is `−0.2` rad.

The identity:
```
e_yaw = atan2(sin(yaw_d - yaw), cos(yaw_d - yaw))
```
always returns the **minimal signed angle** in `(-π, π]`, regardless of the raw values.

### Body Frame Transformation

Exact code (same in all nodes):
```python
ex_b =  math.cos(self.yaw) * ex + math.sin(self.yaw) * ey
ey_b = -math.sin(self.yaw) * ex + math.cos(self.yaw) * ey
```

Matrix form:
```
[ex_b]   [ cos(ψ)  sin(ψ)] [ex]
[ey_b] = [-sin(ψ)  cos(ψ)] [ey]
```

This is `R(-ψ)`, the inverse rotation by current heading `ψ = self.yaw`.

**Physical meaning:** The robot receives velocity commands in its own body frame
(`linear.x` = forward, `linear.y` = left). The body-frame errors `ex_b`, `ey_b`
directly correspond to how far the target is ahead/beside the robot, making the
control law physically meaningful regardless of the robot's current orientation.

---

## 6. PERFORMANCE METRICS AVAILABLE

### Currently Logged (data_logger_node → `smc_data.csv`)

| Column | Description |
|---|---|
| `timestamp` | Wall-clock time (nanoseconds / 1e9, seconds) |
| `x_des`, `y_des` | Desired position from `/target_pose` |
| `x_act`, `y_act` | Actual position from `/odom` |
| `yaw_act` | Actual heading from `/odom` |
| `sliding_surface_x` | Body-frame error `ex_b` (proxy for `s_x`) |
| `sliding_surface_y` | Body-frame error `ey_b` (proxy for `s_y`) |
| `cmd_vel_linear_x` | Published linear x velocity command |
| `cmd_vel_angular_z` | Published angular z velocity command |

Logging rate: **50 Hz**. Log guard: only writes once both `/odom` and `/target_pose`
have been received at least once.

### Topics Available for rosbag2 Recording

```bash
ros2 bag record /odom /cmd_vel /target_pose /joint_states
```

### Suggested KPIs

| Metric | Formula | Interpretation |
|---|---|---|
| RMSE position | `sqrt(mean(ex² + ey²))` | Overall tracking accuracy |
| MAE position | `mean(sqrt(ex² + ey²))` | Robust to outliers |
| Max error | `max(sqrt(ex² + ey²))` | Worst-case deviation |
| Sliding surface RMSE | `sqrt(mean(sx² + sy²))` | How close to manifold |
| Lyapunov function | `V(t) = 0.5 * (sx² + sy²)` | Must be non-increasing for stability |
| Lyapunov derivative | `V̇(t) = numerical diff of V` | Must be ≤ 0 (reaching condition) |
| Chattering index | FFT of `/cmd_vel` — high-freq energy ratio | Lower = smoother control |

### Analysis Scripts (`smc_evaluation/`)

| Script | Output |
|---|---|
| `analyze.py --csv smc_data.csv` | Master runner — all plots + console report |
| `plot_trajectory.py` | 2D x-y path + Euclidean error vs time |
| `plot_errors.py` | 3-subplot ex(t), ey(t), ‖e‖(t) + RMSE/MAE table |
| `plot_sliding.py` | s(t), ‖s‖, V(t), V̇(t) with stability shading |
| `plot_cmdvel.py` | cmd_vel time-domain + FFT chattering analysis |

Saved as `.png` (dpi=300) and `.pdf` in `smc_evaluation/output/`.

---

## 7. PACKAGE STRUCTURE

### Executables (`my_robot_controller`)

| Executable | Source File | Node Name | Purpose |
|---|---|---|---|
| `smc_tracking_node` | `smc_tracking.py` | `smc_tracking_node` | Circular trajectory (R=3 m) |
| `smc_square_node` | `smc_square_node.py` | `smc_square_node` | Square via Lamé curve (n=20, L=3 m) |
| `smc_figure8_node` | `smc_figure8_node.py` | `smc_figure8_node` | Figure-8 lemniscate (A=3 m) |
| `smc_rounded_square_node` | `smc_rounded_square_node.py` | `smc_rounded_square_node` | Rounded square Lamé (n=6, L=3 m) |
| `smc_superellipse_node` | `smc_superellipse_node.py` | `smc_superellipse_node` | Superellipse (n=8, L=1.5 m) |
| `visualizer_node` | `visualizer_node.py` | `visualizer_node` | Real-time matplotlib trajectory plot |
| `data_logger_node` | `data_logger_node.py` | `data_logger_node` | CSV logger at 50 Hz |

### ROS2 Dependencies (`package.xml`)

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

Additional Python runtime (not in package.xml, installed via pip):
- `tf_transformations` — quaternion → euler conversion
- `numpy`, `pandas`, `matplotlib` — analysis toolkit only

### Workspace Packages (`README.md`)

- `my_robot_controller` — SMC controller (this project)
- `unitree_ros2_sim` — Unitree simulation bridge
- `go1_description` — URDF and robot description
- `go1_gazebo` — Gazebo launch and world files
- `go1_navigation` — Nav2 navigation stack integration
- `ros2_unitree_legged_control` — Joint-level leg controller
- `ros2_unitree_legged_msgs` — Custom message types for Unitree

### Directory Layout

```
go1_ws/
├── src/
│   └── my_robot_controller/
│       ├── my_robot_controller/
│       │   ├── smc_tracking.py              ← circular
│       │   ├── smc_square_node.py           ← square (Lamé n=20)
│       │   ├── smc_figure8_node.py          ← figure-8
│       │   ├── smc_rounded_square_node.py   ← rounded square (Lamé n=6)
│       │   ├── smc_superellipse_node.py     ← superellipse (Lamé n=8)
│       │   ├── visualizer_node.py           ← real-time plotter
│       │   └── data_logger_node.py          ← CSV logger
│       ├── setup.py
│       └── package.xml
├── smc_evaluation/
│   ├── analyze.py
│   ├── plot_trajectory.py
│   ├── plot_errors.py
│   ├── plot_sliding.py
│   ├── plot_cmdvel.py
│   ├── utils.py
│   └── output/
│       ├── trajectory.png / .pdf
│       ├── errors.png     / .pdf
│       ├── sliding.png    / .pdf
│       └── cmdvel.png     / .pdf
└── smc_data.csv                             ← logged experiment data
```

---

*End of PROJECT_SUMMARY.md*
