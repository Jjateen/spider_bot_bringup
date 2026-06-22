cmake_minimum_required(VERSION 3.8)
project(big_bertha_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install all bringup assets into the package share directory. RViz +
# PlotJuggler configs live under config/ (config/rviz, config/plotjuggler).
install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
