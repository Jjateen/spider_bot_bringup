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

# ----------------------------- ONNX Runtime auto-download -------------------
# ONNX Runtime (C++) is not packaged via apt/rosdep on Jazzy. If the prebuilt
# release is not already present under ONNXRUNTIME_ROOT, CMake downloads it
# automatically during configure, matching the host CPU architecture.
set(ORT_VER "1.20.1")

# Map CMake system processor to the arch label used in ONNX Runtime release
# tarballs (https://github.com/microsoft/onnxruntime/releases).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(ORT_ARCH "x64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ORT_ARCH "aarch64")
else()
  message(FATAL_ERROR
    "Unsupported CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT DEFINED ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.onnxruntime"
      CACHE PATH "ONNX Runtime install prefix")
endif()

# Skip download if headers + library already exist
set(ORT_REQUIRED_HEADER "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
set(ORT_REQUIRED_LIB "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

if(NOT EXISTS "${ORT_REQUIRED_HEADER}" OR NOT EXISTS "${ORT_REQUIRED_LIB}")
  message(STATUS
    "ONNX Runtime not found at ${ONNXRUNTIME_ROOT} — "
    "downloading v${ORT_VER} for ${ORT_ARCH}")

  set(ORT_URL
    "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VER}/onnxruntime-linux-${ORT_ARCH}-${ORT_VER}.tgz")
  set(ORT_TGZ "${CMAKE_BINARY_DIR}/onnxruntime-linux-${ORT_ARCH}-${ORT_VER}.tgz")

  file(DOWNLOAD "${ORT_URL}" "${ORT_TGZ}"
       SHOW_PROGRESS STATUS ORT_DL_STATUS)
  list(GET ORT_DL_STATUS 0 ORT_DL_RET)
  if(ORT_DL_RET)
    message(FATAL_ERROR
      "Failed to download ONNX Runtime from ${ORT_URL}: ${ORT_DL_STATUS}")
  endif()

  file(MAKE_DIRECTORY "${ONNXRUNTIME_ROOT}")
  execute_process(
    COMMAND tar xzf "${ORT_TGZ}" --strip-components=1 -C "${ONNXRUNTIME_ROOT}"
    RESULT_VARIABLE ORT_TAR_RESULT)
  if(NOT ORT_TAR_RESULT EQUAL 0)
    message(FATAL_ERROR
      "Failed to extract ${ORT_TGZ}: tar returned ${ORT_TAR_RESULT}")
  endif()

  file(REMOVE "${ORT_TGZ}")
  message(STATUS "ONNX Runtime extracted to ${ONNXRUNTIME_ROOT}")
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include
  PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR
    "ONNX Runtime not found under '${ONNXRUNTIME_ROOT}'. "
    "Download v${ORT_VER} and pass -DONNXRUNTIME_ROOT=<path> "
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
  # clang-format (.clang-format + the pre-commit hook) is the canonical C++
  # style for this repo. ament_uncrustify uses a different, conflicting style
  # (constructor-initializer and continuation alignment), so skip it here and
  # let clang-format be the single source of truth; cpplint and the remaining
  # linters still run.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
