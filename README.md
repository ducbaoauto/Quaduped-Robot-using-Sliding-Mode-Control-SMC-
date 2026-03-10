ROS2 workspace for Unitree GO1 simulation and custom control packages.

Main packages currently in this workspace include:

- `my_robot_controller`
- `unitree_ros2_sim`
- `go1_description`
- `go1_gazebo`
- `go1_navigation`
- `ros2_unitree_legged_control`
- `ros2_unitree_legged_msgs`

## Requirements

- Ubuntu (tested on Linux)
- ROS2 (Humble installed distro)
- `colcon`
- `rosdep`
- Python 3

## Quick start

### 1) Clone

```bash
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
```

### 2) Install dependencies

```bash
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3) Build

```bash
source /opt/ros/<your-ros2-distro>/setup.bash
colcon build --symlink-install
```

### 4) Source workspace

```bash
source install/setup.bash
```

## Typical development flow

```bash
# from workspace root
source /opt/ros/<your-ros2-distro>/setup.bash
colcon build --packages-select my_robot_controller --symlink-install
source install/setup.bash
```

## Run workflow (verified)

This is the tested run order for this project.

### 0) Open terminal in workspace root

```bash
cd /home/ducbao/go1_ws
```

### 1) Build workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you only changed your controller package:

```bash
colcon build --packages-select my_robot_controller --symlink-install
source install/setup.bash
```

### 2) Start simulation (Terminal 1)

```bash
cd /home/ducbao/go1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go1_gazebo spawn_go1.launch.py
```

Optional world file:

```bash
ros2 launch go1_gazebo spawn_go1.launch.py world_file_name:=test_latest.world
```

### 3) Start Unitree controller interface (Terminal 2)

Wait until all controllers are loaded (especially `RL_calf_controller`), then run:

```bash
cd /home/ducbao/go1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run unitree_guide2 junior_ctrl
```

Inside `junior_ctrl`:
- press `2` for stand mode
- press `5` for move_base mode (accepts `/cmd_vel`)

### 4) Run custom SMC nodes

Terminal 3:

```bash
cd /home/ducbao/go1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_controller smc_tracking_node
```

Terminal 4:

```bash
cd /home/ducbao/go1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_controller visualizer_node
```

Alternative controller:

```bash
ros2 run my_robot_controller smc_square_node
```

### 5) Optional navigation stack

```bash
cd /home/ducbao/go1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go1_navigation navigation.launch.py
```

### 6) Useful checks

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 pkg executables my_robot_controller
```

If command not found, re-source in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/ducbao/go1_ws/install/setup.bash
```

## Attribution

- This workspace includes and adapts open-source ROS 2 / Unitree simulation and support packages.
- The Sliding Mode Control (SMC) logic in `my_robot_controller` is original work by this project author.
- Please keep upstream licenses and credit notices when reusing or redistributing.


## License

