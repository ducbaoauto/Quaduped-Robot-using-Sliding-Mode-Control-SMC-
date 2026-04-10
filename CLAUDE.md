# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble project implementing **Sliding Mode Control (SMC)** for trajectory tracking on the Unitree GO1 quadruped robot in Gazebo simulation. The main controller package is `my_robot_controller` (Python); the rest of the packages (`unitree_ros2_sim/`) are simulation infrastructure.

## Build Commands

```bash
# Source ROS2 first (required before any colcon command)
source /opt/ros/humble/setup.bash

# Build entire workspace
colcon build --symlink-install

# Build only the controller package (faster iteration)
colcon build --packages-select my_robot_controller --symlink-install

# Source workspace after build
source install/setup.bash
```

`--symlink-install` means Python source edits take effect without rebuilding.

## Run Workflow

Four terminals, started in order:

**Terminal 1 — Gazebo simulation:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch go1_gazebo spawn_go1.launch.py
```

**Terminal 2 — Unitree control interface (after `RL_calf_controller` appears in Terminal 1):**
```bash
ros2 run unitree_guide2 junior_ctrl
# Inside: press 2 (stand), then 5 (move_base / accept /cmd_vel)
```

**Terminal 3 — SMC controller (choose one):**
```bash
ros2 run my_robot_controller smc_tracking_node        # circular
ros2 run my_robot_controller smc_square_node          # Lamé n=20 square
ros2 run my_robot_controller smc_figure8_node         # lemniscate figure-8
ros2 run my_robot_controller smc_rounded_square_node  # Lamé n=6 rounded square
ros2 run my_robot_controller smc_superellipse_node    # Lamé n=8 superellipse
```

**Terminal 4 — Optional tools:**
```bash
ros2 run my_robot_controller visualizer_node    # real-time matplotlib plot
ros2 run my_robot_controller data_logger_node   # CSV log → smc_data.csv at 50 Hz
```

## Analysis Scripts

After a run, evaluate performance from `smc_evaluation/`:
```bash
python3 smc_evaluation/analyze.py --csv smc_data.csv   # master runner: all plots + report
python3 smc_evaluation/plot_trajectory.py              # 2D path + error vs time
python3 smc_evaluation/plot_errors.py                  # ex(t), ey(t), ‖e‖(t)
python3 smc_evaluation/plot_sliding.py                 # sliding surface + Lyapunov function
python3 smc_evaluation/plot_cmdvel.py                  # cmd_vel + FFT chattering analysis
```
Outputs go to `smc_evaluation/output/` as PNG (300 dpi) and PDF.

## Architecture

### ROS2 Topic Graph

```
Gazebo (/odom) → SMC node → /cmd_vel → Robot (plant)
                          → /target_pose → visualizer_node
/odom → data_logger_node ← /target_pose, /cmd_vel
```

Key topics: `/odom` (`nav_msgs/Odometry`), `/cmd_vel` (`geometry_msgs/Twist`), `/target_pose` (`geometry_msgs/PoseStamped`).

### SMC Control Law

All SMC nodes share the same pattern and gains:

```
State: [x, y, yaw] from /odom
Error in global frame → rotate to body frame:
  ex_b =  cos(yaw)*ex + sin(yaw)*ey
  ey_b = -sin(yaw)*ex + cos(yaw)*ey

Control (smooth SMC with tanh boundary layer, no chattering):
  v_x = k_v * tanh(ex_b  / phi)   # k_v=0.5, phi=0.5
  v_y = k_v * tanh(ey_b  / phi)
  w_z = k_w * tanh(e_yaw / phi)   # k_w=1.5
```

Gains are identical across all five node files. Yaw errors use `atan2(sin·, cos·)` for safe [-π, π] wrapping.

### Node Frequencies

| Node | Rate |
|------|------|
| SMC controller nodes | 20 Hz |
| `data_logger_node` | 50 Hz |
| `visualizer_node` | ~10 Hz |

### Package Layout

- `src/my_robot_controller/` — Python SMC nodes (all trajectory variants, visualizer, logger)
- `src/unitree_ros2_sim/go1_description/` — URDF/Xacro robot model
- `src/unitree_ros2_sim/go1_gazebo/` — Gazebo launch files and worlds
- `src/unitree_ros2_sim/unitree_guide2/` — C++ control state machine
- `src/unitree_ros2_sim/ros2_unitree_legged_controller/` — C++ joint control plugin
- `src/unitree_ros2_sim/ros2_unitree_legged_msgs/` — Custom Unitree message types
- `smc_evaluation/` — Offline analysis scripts

### Adding a New Trajectory Node

1. Copy an existing node (e.g., `smc_tracking.py`) as a starting point.
2. Override `get_desired_trajectory(t)` to return `(xd, yd, yaw_d)`.
3. Handle derivative singularities with a 1e-6 guard (see `smc_square_node.py`).
4. Register the entry point in `my_robot_controller/setup.py`.
5. Rebuild: `colcon build --packages-select my_robot_controller --symlink-install`.
