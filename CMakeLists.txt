cmake_minimum_required(VERSION 3.16)
project(amr_slam_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ─── C++ obstacle spawner node ───────────────────────────────────────
add_executable(dynamic_obstacle_node src/dynamic_obstacle_node.cpp)
ament_target_dependencies(dynamic_obstacle_node
  rclcpp geometry_msgs nav_msgs tf2_ros)
target_compile_features(dynamic_obstacle_node PRIVATE cxx_std_17)

install(TARGETS dynamic_obstacle_node DESTINATION lib/${PROJECT_NAME})

# ─── Python package ──────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ─── Python scripts ──────────────────────────────────────────────────
install(PROGRAMS
  scripts/waypoint_navigator.py
  scripts/run_waypoint_mission.py
  scripts/benchmark_navigation.py
  scripts/map_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Data files ──────────────────────────────────────────────────────
install(DIRECTORY
  launch config maps urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

# ─── Tests ───────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_waypoint test/test_waypoint_logic.py)
  ament_add_pytest_test(test_benchmarks test/test_benchmarks.py)
endif()

ament_package()
