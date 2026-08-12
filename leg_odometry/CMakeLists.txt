cmake_minimum_required(VERSION 3.8)
project(leg_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Built as a component library so the node can be loaded into the composed
# container alongside policy_controller and hardware_bridge, where
# /joint_states, /odom and /position_controller/commands ride intra-process
# and skip serialisation. rclcpp_components_register_node also generates the
# standalone legged_odometry_node executable from the same library, so
# launching it on its own keeps working exactly as before.
add_library(legged_odometry_component SHARED src/legged_odometry_node.cpp)
ament_target_dependencies(legged_odometry_component
  rclcpp rclcpp_components std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

rclcpp_components_register_node(legged_odometry_component
  PLUGIN "leg_odometry::LeggedOdometryNode"
  EXECUTABLE legged_odometry_node)

ament_export_targets(export_legged_odometry_component)
install(TARGETS legged_odometry_component
  EXPORT export_legged_odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
