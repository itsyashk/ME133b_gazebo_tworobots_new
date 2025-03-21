# ME133b Two-Robot SLAM Simulation

This repository contains a complete ROS2 workspace for running a two-robot SLAM simulation in Gazebo. The simulation features two differential-drive robots equipped with LiDAR sensors exploring a Mars-like environment.

## Features

- Two independent differential-drive robots
- LiDAR-based SLAM for mapping
- Mars-like Gazebo world
- Independent keyboard control for each robot
- RViz visualization
- Complete launch system with error handling

## Prerequisites

- Ubuntu 22.04 (WSL2 supported)
- ROS2 Humble
- Gazebo 11
- SLAM Toolbox
- Navigation2

## Workspace Structure

```
ME133b_gazebo_tworobots/
├── mars_robots/          # Robot description package
├── mars_simulation/      # Main simulation package
├── mars_world/          # World description package
└── launch_scripts/      # Helper scripts
```

## Quick Start

1. Clone the repository:
```bash
git clone https://github.com/itsyashk/ME133b_gazebo_tworobots_new.git
cd ME133b_gazebo_tworobots_new
```

2. Build the workspace:
```bash
colcon build
source install/setup.bash
```

3. Launch the simulation:
```bash
./launch_fixed_mars.sh
```

4. In separate terminals, control the robots:
```bash
# Robot 1 (WASD keys)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel

# Robot 2 (Arrow keys)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot2/cmd_vel
```

## License

Apache 2.0