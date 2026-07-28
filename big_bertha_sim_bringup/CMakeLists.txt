cmake_minimum_required(VERSION 3.8)
project(big_bertha_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Install all bringup assets into the package share directory. RViz +
# PlotJuggler configs live under config/ (config/rviz, config/plotjuggler).
install(
  DIRECTORY launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

# Perception helper node (scan_ground_filter: IMU-gated ghost-wall removal).
add_executable(scan_ground_filter src/scan_ground_filter.cpp)
target_include_directories(scan_ground_filter PRIVATE include)
ament_target_dependencies(scan_ground_filter
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)
install(
  TARGETS scan_ground_filter
  DESTINATION lib/${PROJECT_NAME}
)

# maps/ is added by a later chained PR (mapping/localization); install it
# only when present so earlier-chain branches still build.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(
    DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  # Ground-cull sign conventions (the historical inversion deleted real walls).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ground_ray test/test_ground_ray.cpp)
  target_include_directories(test_ground_ray PRIVATE include)

  # Frontier maths behind the SLAM demo: it also decides when the map is
  # complete, so a wrong "no frontiers" answer ends exploration early.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_frontier test/test_frontier.py)

  find_package(ament_lint_auto REQUIRED)
  # The world SDF and saved maps are generated/3rd-party formatting; skip
  # copyright/lint on data dirs, lint only launch + config python/yaml.
  set(ament_cmake_copyright_FOUND TRUE)
  # clang-format (.clang-format + the pre-commit hook) is the canonical C++ style
  # for this repo; ament_uncrustify uses a different, conflicting style (e.g. it
  # wraps constructor initializer lists clang-format keeps inline). Disable it so
  # clang-format stays the single source of truth (same as big_bertha_policy_controller).
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
