# Mars Robots Simulation

This ROS 2 package provides a simulation environment for two robots operating on a Mars-like terrain using Gazebo. The simulation includes differential drive robots equipped with laser scanners and cameras, operating in a custom Mars world environment.

## Features

- Two identical differential drive robots with laser scanners and cameras
- Custom Mars-like Gazebo world with appropriate gravity and terrain features
- Navigation capabilities using ROS 2 Navigation Stack (Nav2)
- Sensor integration including laser scanners and cameras
- Proper namespacing for multi-robot operation
- RViz configuration for visualization

## Prerequisites

- ROS 2 Humble or later
- Gazebo
- Navigation2 (Nav2)
- xacro
- robot_state_publisher
- joint_state_publisher
- rviz2

## Installation

1. Create a ROS 2 workspace (if you haven't already):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/itsyashk/ME133b_gazebo_tworobots_new.git
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build
```

5. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Launch the simulation with two robots:
```bash
ros2 launch mars_robots two_robots_sim.launch.py
```

This will start:
- Gazebo with the Mars world
- Two robot instances
- RViz for visualization
- Navigation stack for both robots

## Robot Control

Each robot can be controlled independently using the following topics:

### Robot 1
- Velocity commands: `/robot1/cmd_vel`
- Laser scan data: `/robot1/scan`
- Camera feed: `/robot1/camera/image_raw`
- Odometry: `/robot1/odom`

### Robot 2
- Velocity commands: `/robot2/cmd_vel`
- Laser scan data: `/robot2/scan`
- Camera feed: `/robot2/camera/image_raw`
- Odometry: `/robot2/odom`

## Configuration Files

- `urdf/mars_robot.urdf.xacro`: Robot description file
- `urdf/mars_robot.gazebo.xacro`: Gazebo-specific robot configurations
- `worlds/mars.world`: Gazebo world file with Mars-like environment
- `config/nav2_params.yaml`: Navigation stack parameters
- `config/rviz_params.yaml`: RViz configuration parameters
- `launch/two_robots_sim.launch.py`: Main launch file

## Project Structure

```
mars_robots/
├── config/
│   ├── nav2_params.yaml
│   └── rviz_params.yaml
├── launch/
│   └── two_robots_sim.launch.py
├── urdf/
│   ├── mars_robot.urdf.xacro
│   └── mars_robot.gazebo.xacro
├── worlds/
│   └── mars.world
├── rviz/
│   └── urdf.rviz
├── CMakeLists.txt
└── package.xml
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 Community
- Gazebo Simulator
- Navigation2 Team