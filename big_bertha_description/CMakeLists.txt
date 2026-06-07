cmake_minimum_required(VERSION 3.8)
project(big_bertha_description)

find_package(ament_cmake REQUIRED)

# Install model assets and launch files into the package share directory.
install(
  DIRECTORY urdf config launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
