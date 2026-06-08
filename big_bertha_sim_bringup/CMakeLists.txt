cmake_minimum_required(VERSION 3.8)
project(big_bertha_sim_bringup)

find_package(ament_cmake REQUIRED)

# Install all bringup assets into the package share directory.
install(
  DIRECTORY launch config worlds rviz maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The world SDF and saved maps are generated/3rd-party formatting; skip
  # copyright/lint on data dirs, lint only launch + config python/yaml.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
