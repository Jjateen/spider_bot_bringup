cmake_minimum_required(VERSION 3.8)
project(big_bertha_policy_controller)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spider_msgs REQUIRED)

# ----------------------------- ONNX Runtime -------------------------------
# Prebuilt ONNX Runtime (C++), located via -DONNXRUNTIME_ROOT (defaults to the
# repo-local .onnxruntime download). Not provided by rosdep/apt on Jazzy.
if(NOT DEFINED ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.onnxruntime"
      CACHE PATH "ONNX Runtime install prefix")
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include
  PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR
    "ONNX Runtime not found under '${ONNXRUNTIME_ROOT}'. "
    "Download v1.20.1 and pass -DONNXRUNTIME_ROOT=<path> "
    "(see PLAN.md / repo README).")
endif()
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIB}")

get_filename_component(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_LIB}" DIRECTORY)

add_executable(policy_controller_node src/policy_controller_node.cpp)
target_include_directories(policy_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(policy_controller_node ${ONNXRUNTIME_LIB})
ament_target_dependencies(policy_controller_node
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs spider_msgs)

# Embed the ONNX Runtime lib directory in the binary RPATH so the shared
# library resolves at runtime without LD_LIBRARY_PATH (the prebuilt ORT is
# not on the system loader path).
set_target_properties(policy_controller_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS policy_controller_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ONNX Runtime headers are third-party; skip uncrustify/cpplint include
  # ordering noise on them by linting our sources only via default config.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
