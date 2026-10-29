
# Robot Motion Planning

### Ackermann Vehicle Motion Planning and Trajectory Tracking

A C++17-based educational and experimental project for Ackermann-steered vehicle motion planning, featuring A*, Hybrid A*, vehicle kinematics, and ROS 2 simulation.

This repository documents my learning and implementation of robot motion planning algorithms, from grid-based path searching to kinematically feasible planning.

**Technical Blog:** [Robot Motion Planning & Navigation Notes](https://zzy412.github.io/robot-navigation-blog/)

---

## 1. Project Overview

Autonomous navigation for Ackermann-steered vehicles requires more than collision-free geometric paths. Vehicle heading, steering-angle limits, and turning radius must also be considered.

This project aims to study and compare classical path-planning algorithms and develop a modular simulation framework.

The project currently consists of two components:

- **Standalone C++ simulation:** A* and Hybrid A* planning with experimental data output.
- **ROS 2 integration:** A Hybrid A* planning node and an Ackermann vehicle kinematic simulation node.

The project is intended for education, simulation, and algorithm development. It is not a verified real-world autonomous driving system.

## 2. Main Features

### Path Planning

- Grid-based A* path planning
- Hybrid A* planning using Ackermann vehicle kinematics
- Steering-based motion primitive generation
- Obstacle collision checking
- Path and planning-performance analysis

### Vehicle Simulation and ROS 2

- Kinematic bicycle model
- ROS 2 Hybrid A* planning node
- ROS 2 Ackermann vehicle simulation node
- Occupancy-grid and path publishing
- Odometry and vehicle visualization in RViz2

### Related Experiments

Additional local C++ experiments have investigated:

- Pure Pursuit path tracking
- Fixed and adaptive lookahead strategies
- Steering-rate constraints
- Curvature-aware speed planning
- Multi-scenario performance comparisons

Some of these later experimental implementations are being organized for public release and should not be assumed to be included in the standalone code currently published.

## 3. Technology Stack

| Component | Technology |
|---|---|
| Programming language | C++17 |
| Build system | CMake |
| Windows IDE | Visual Studio 2022 |
| Linux environment | Ubuntu 24.04 |
| Robotics middleware | ROS 2 Jazzy |
| ROS 2 build tool | colcon |
| Visualization | RViz2, CSV-based experiment analysis |
| Version control | Git and GitHub |

## 4. Repository Structure

```text
Robot-Motion-Planning/
├── include/                   # C++ header files
├── src/                       # C++ algorithm implementations
├── results/                   # Experiment results
├── ros2/
│   └── ackermann_navigation/
│       ├── include/           # ROS 2 package headers
│       ├── src/               # ROS 2 node implementations
│       ├── CMakeLists.txt
│       └── package.xml
├── CMakeLists.txt             # Standalone C++ build configuration
├── CMakePresets.json
├── main.cpp                   # Standalone program entry
├── .gitignore
└── README.md
```

## 5. Algorithms

### 5.1 A* Search

The A* algorithm uses an evaluation function:

$$
f(n)=g(n)+h(n)
$$

where:

- `g(n)` is the accumulated cost from the start.
- `h(n)` is the estimated cost to the goal.
- `f(n)` is the estimated total cost.

The implementation serves as a geometric path-planning baseline.

### 5.2 Hybrid A*

Unlike grid-based A*, Hybrid A* considers vehicle orientation and generates successor states by simulating feasible steering motions.

A simplified kinematic bicycle model is used:

$$
\begin{aligned}
\dot{x} &= v\cos\theta \\
\dot{y} &= v\sin\theta \\
\dot{\theta} &= \frac{v}{L}\tan\delta
\end{aligned}
$$

where:

- `x, y`: vehicle position
- `theta`: heading angle
- `v`: longitudinal speed
- `L`: wheelbase
- `delta`: front steering angle

The steering-angle limit introduces a minimum turning radius:

$$
R_{\min}=\frac{L}{\tan(\delta_{\max})}
$$

This makes Hybrid A* more appropriate than geometric A* for generating reference paths for Ackermann-steered vehicles.

## 6. Build and Run: Standalone C++

### Option A: Visual Studio 2022

1. Install Visual Studio 2022 with **Desktop development with C++**.
2. Open Visual Studio.
3. Select **File > Open > Folder**.
4. Choose the repository root containing `CMakeLists.txt`.
5. Wait for CMake configuration.
6. Select `RobotMotionPlanning.exe` as the startup target.
7. Run using `Ctrl + F5`.

### Option B: CMake Command Line

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

On a typical Linux single-configuration build:

```bash
./build/RobotMotionPlanning results
```

On a Windows Visual Studio multi-configuration build, the executable is commonly located under:

```text
build/Release/RobotMotionPlanning.exe
```

The exact executable path depends on the CMake generator and build configuration.

## 7. ROS 2 Jazzy Simulation

The ROS 2 package is stored at:

```text
ros2/ackermann_navigation
```

### 7.1 ROS 2 Dependencies

- `ament_cmake`
- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `ackermann_msgs`
- `tf2_ros`
- `visualization_msgs`

### 7.2 Build the ROS 2 Package

On Ubuntu 24.04 with ROS 2 Jazzy installed, create a workspace:

```bash
mkdir -p ~/ackermann_ws/src
cd ~/ackermann_ws/src
```

Copy the `ackermann_navigation` directory from this repository into `src`.

Build the package:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ackermann_ws

colcon build --packages-select ackermann_navigation
source install/setup.bash
```

### 7.3 Run the Planner Node

```bash
ros2 run ackermann_navigation planner_node
```

### 7.4 Run the Vehicle Simulation Node

In a second ROS 2 terminal:

```bash
ros2 run ackermann_navigation vehicle_sim_node
```

### 7.5 Launch RViz2

In another terminal:

```bash
rviz2
```

Set the RViz2 Fixed Frame to:

```text
map
```

Configure the relevant displays using the published topics.

### 7.6 ROS 2 Topics

| Topic | Message Type | Function |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy-grid map |
| `/planned_path` | `nav_msgs/msg/Path` | Hybrid A* reference path |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Vehicle motion command |
| `/odom` | `nav_msgs/msg/Odometry` | Simulated vehicle odometry |
| `/actual_path` | `nav_msgs/msg/Path` | Simulated vehicle trajectory |
| `/vehicle_marker` | `visualization_msgs/msg/Marker` | RViz2 vehicle visualization |

The current ROS 2 simulation supports planner visualization and externally issued drive commands. An online closed-loop Pure Pursuit controller is a planned development task.

## 8. Experiment Outputs

The standalone C++ project generates data for inspecting planning performance.

Typical output files include:

| File | Description |
|---|---|
| `vehicle_paths.csv` | Fixed-steering vehicle trajectories |
| `obstacles.csv` | Obstacle geometry |
| `astar_path.csv` | A* path |
| `hybrid_path.csv` | Hybrid A* path |
| `metrics.csv` | Planning-performance measurements |

The exact output location depends on the program's working directory or the output-directory argument.

## 9. Limitations

This repository is intended for educational and experimental use.

Important current limitations include:

- The standalone planner uses simplified vehicle geometry.
- Collision checking is based on discretely sampled vehicle states.
- Continuous swept-volume collision checking is not yet included.
- Steering-rate and acceleration constraints are not enforced by the original standalone planning baseline.
- The basic A* path is a geometric reference, not necessarily a directly drivable trajectory.
- Hybrid A* is not claimed to provide a globally optimal solution.
- The current ROS 2 system does not yet implement closed-loop tracking of the published path.

These limitations should be considered when interpreting simulation results.

## 10. Future Work

Planned improvements include:

1. Online Pure Pursuit tracking in ROS 2.
2. Steering-rate-constrained motion planning.
3. Curvature-aware velocity planning.
4. Additional planning and control baselines.
5. More complex obstacle environments.
6. Improved collision checking and vehicle geometry.
7. Reproducible benchmarking and experiment visualization.
8. Integration with a physical Ackermann vehicle platform.

## 11. Blog and Learning Resources

**My Technical Blog:**

https://zzy412.github.io/robot-navigation-blog/

Related open-source projects:

- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)
- [PathFinding.js](https://github.com/qiao/PathFinding.js)
- [Hybrid A* Path Planner](https://github.com/karlkurzer/path_planner)
- [OMPL](https://github.com/ompl/ompl)
- [ROS 2 Navigation2](https://github.com/ros-navigation/navigation2)

---

**Project Status:** Ongoing development

**Main Focus:** Ackermann Motion Planning, Hybrid A*, Trajectory Tracking, and ROS 2
