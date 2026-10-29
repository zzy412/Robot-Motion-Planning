
cmake_minimum_required(VERSION 3.8)
project(ackermann_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Existing Hybrid A* planner
add_executable(planner_node
  src/planner_node.cpp
  src/VehicleModel.cpp
  src/GridMap.cpp
  src/AStar.cpp
  src/HybridAStar.cpp
)

target_include_directories(planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# New Ackermann vehicle simulator
add_executable(vehicle_sim_node
  src/vehicle_sim_node.cpp
  src/VehicleModel.cpp
)

target_include_directories(vehicle_sim_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(vehicle_sim_node
  rclcpp
  nav_msgs
  geometry_msgs
  ackermann_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  planner_node
  vehicle_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

