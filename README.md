# Robot Motion Planning (C++17 + CMake)

A self-contained educational Ackermann vehicle / A* / Hybrid A* comparison. No ROS dependency. Designed for Visual Studio 2022 CMake Project, and portable to Linux.

## Visual Studio 2022
1. Install `Desktop development with C++` including CMake tools.
2. **File → Open → Folder**, choose this project root (the one containing `CMakeLists.txt`).
3. Wait for CMake configure. Select `RobotMotionPlanning.exe` startup target, then **Ctrl+F5**.
4. Output CSVs are written into a `results` directory relative to the process working directory (shown on screen). Set command-line argument to an absolute output directory to control its location.

## Terminal alternative
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/RobotMotionPlanning results
```
For MSVC multi-config generators use `build/Release/RobotMotionPlanning.exe` and supply output dir as argument.

## Output
- `vehicle_paths.csv`: 5 fixed-steering trajectories
- `obstacles.csv`: 3 rectangle obstacles
- `astar_path.csv`: grid-based 8-neighbor A* path
- `hybrid_path.csv`: sampled Ackermann bicycle motion path
- `metrics.csv`: planning time, success, length, approximate curvature, center clearance and expanded states

## Important limitations
This is an educational planner, **not a vehicle controller or a verified safe system**. Uses constant-speed steering primitives and a conservative body rectangle test at discrete trajectory samples. Robot pose is taken at rectangle center for demonstration; the real rear-axle reference point and front/rear overhang require calibration. Does not check steering rate, acceleration, dynamic obstacles or swept-volume continuous collisions. A* uses an approximate center-radius collision envelope and is a geometric baseline, not a driveable Ackermann trajectory. Hybrid A* terminates within pose tolerances rather than an exact analytic goal connection; heuristic includes heading bias and therefore no optimality claim. The `min_center_clearance` metric is for the **path reference point**, not the vehicle body, and should not be presented as body clearance.

## Next development milestones
1. Replace hardcoded teaching parameters with measured vehicle geometry.
2. Add heading-aware Reeds–Shepp analytical expansions, steering rate constraints, continuous collision checking and better heuristics.
3. Test many start/goal poses and random seeds (if sampling randomness is introduced).
4. Wrap core planner as ROS 2 `ament_cmake` library and publish `nav_msgs/msg/Path`, with poses converted to yaw quaternions.
