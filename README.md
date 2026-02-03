# Adaptive Maze Navigator

**ROS 2 Autonomous Navigation System** | University of Malta | ARI3215 Robotics 2

A TurtleBot3 Waffle robot navigating autonomously through a maze environment using the Nav2 navigation stack, demonstrating the AI perception-reasoning-action loop.

## Demo

The robot uses LiDAR sensing and AMCL localization to navigate from point A to point B without human control.

## Features

- **Autonomous Navigation** — Nav2 path planning and trajectory execution
- **AMCL Localization** — Particle filter-based pose estimation
- **2 Sensors** — 360° LiDAR + IMU
- **Simulation** — Gazebo Harmonic with custom maze world

## Tech Stack

| Component | Technology |
|-----------|------------|
| OS | Ubuntu 24.04 ARM64 |
| ROS | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Harmonic |
| Robot | TurtleBot3 Waffle |
| Navigation | Nav2 (AMCL + DWB Controller) |
| Mapping | slam_toolbox / Cartographer |

## Project Structure

```
├── worlds/
│   └── maze_world_with_robot.sdf    # Gazebo maze environment
├── maps/
│   ├── maze_map_clean.yaml          # Navigation map metadata
│   └── maze_map_clean.pgm           # Occupancy grid image
├── launch_demo.sh                   # One-click demo launcher
└── README.md
```

## Quick Start

### Prerequisites

```bash
# ROS 2 Jazzy + Nav2 + TurtleBot3 packages
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-turtlebot3* ros-jazzy-ros-gz
```

### Run the Demo

```bash
# Make executable and run
chmod +x launch_demo.sh
./launch_demo.sh
```

Wait ~60 seconds for full initialization, then run:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

### Manual Control

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## System Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Gazebo    │────▶│  ros_gz     │────▶│    Nav2     │
│  Simulator  │     │   Bridge    │     │   Stack     │
└─────────────┘     └─────────────┘     └─────────────┘
      │                                        │
      ▼                                        ▼
┌─────────────┐                         ┌─────────────┐
│   Sensors   │                         │    AMCL     │
│ LiDAR + IMU │                         │ Localization│
└─────────────┘                         └─────────────┘
```

## Challenges Overcome

1. **VM Environment** — Gazebo GUI too slow → Headless mode solution
2. **SLAM Drift** — Odometry drift corrupted maps → Programmatic map generation
3. **Message Type Mismatch** — TwistStamped vs Twist → Explicit bridge config
4. **Map-World Alignment** — Pose sync between Gazebo and Nav2

## Author

Liam Debono — University of Malta, Faculty of ICT

## License

MIT License — Academic project for ARI3215 Robotics 2
