cmake_minimum_required(VERSION 3.8)
project(big_bertha_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install all bringup assets into the package share directory. RViz +
# PlotJuggler configs live under config/ (config/rviz, config/plotjuggler).
# USE_SOURCE_PERMISSIONS: plain install(DIRECTORY) drops the executable bit on
# scripts/*.sh (send_patrol.sh, dds_env.sh, roudi.sh), breaking direct
# invocation from the install tree.
install(
  DIRECTORY launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Perception helper node (scan_ground_filter: IMU-gated ghost-wall removal).
add_executable(scan_ground_filter src/scan_ground_filter.cpp)
ament_target_dependencies(scan_ground_filter rclcpp sensor_msgs)
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
