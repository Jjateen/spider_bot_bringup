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
# Prebuilt ONNX Runtime (C++). It is NOT on rosdep/apt for Jazzy, so it cannot
# be a package.xml dependency (#39). Resolution order: (1) -DONNXRUNTIME_ROOT or
# the repo-local .onnxruntime; (2) otherwise AUTO-FETCH the official prebuilt for
# this CPU arch so a plain `colcon build` works out-of-the-box -- including on
# aarch64 targets like the Arduino Uno Q (no manual download step).
set(ORT_VERSION "1.20.1" CACHE STRING "ONNX Runtime version")
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
  # Not found locally -> download the official prebuilt for this CPU arch so
  # the build is self-contained (fixes #39: fresh/local/hardware builds failed
  # with no ORT and no -DONNXRUNTIME_ROOT).
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ORT_ARCH "aarch64")
  else()
    set(ORT_ARCH "x64")
  endif()
  set(ORT_PKG "onnxruntime-linux-${ORT_ARCH}-${ORT_VERSION}")
  message(STATUS
    "ONNX Runtime not found under '${ONNXRUNTIME_ROOT}'; auto-fetching ${ORT_PKG} ...")
  include(FetchContent)
  FetchContent_Declare(onnxruntime
    URL "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_PKG}.tgz")
  FetchContent_MakeAvailable(onnxruntime)
  set(ONNXRUNTIME_ROOT "${onnxruntime_SOURCE_DIR}")
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_ROOT}/include
    PATH_SUFFIXES onnxruntime onnxruntime/core/session)
  find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)
endif()

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR
    "ONNX Runtime unavailable: not found under '${ONNXRUNTIME_ROOT}' and the "
    "auto-fetch of v${ORT_VERSION} failed (network is needed on the first build). "
    "Pass -DONNXRUNTIME_ROOT=<path> to a prebuilt install instead.")
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
  # clang-format (.clang-format + the pre-commit hook) is the canonical C++
  # style for this repo. ament_uncrustify uses a different, conflicting style
  # (constructor-initializer and continuation alignment), so skip it here and
  # let clang-format be the single source of truth; cpplint and the remaining
  # linters still run.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
