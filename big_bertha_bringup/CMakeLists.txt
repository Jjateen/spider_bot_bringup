cmake_minimum_required(VERSION 3.8)
project(big_bertha_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Hardware bridge node: ROS 2 node that talks to the arduino-router
# MsgPack-RPC unix socket (Bridge RPC) — no TCP/Python relay.
add_executable(hardware_bridge_node src/hardware_bridge_node.cpp)
target_include_directories(hardware_bridge_node PRIVATE include src)
ament_target_dependencies(hardware_bridge_node rclcpp std_msgs sensor_msgs std_srvs)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
