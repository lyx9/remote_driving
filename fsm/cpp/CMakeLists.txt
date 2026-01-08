cmake_minimum_required(VERSION 3.16)
project(fsm_pilot VERSION 1.1.0 LANGUAGES CXX)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# =============================================================================
# 解决 Anaconda 与系统库冲突
# =============================================================================
# 如果检测到 Anaconda 环境，排除其库路径以避免 Protobuf/gRPC 冲突
if(DEFINED ENV{CONDA_PREFIX} OR EXISTS "/opt/anaconda3")
  message(STATUS "Detected Anaconda installation, excluding from library search")
  # 从搜索路径中排除 Anaconda
  list(FILTER CMAKE_PREFIX_PATH EXCLUDE REGEX "anaconda|conda")
  list(FILTER CMAKE_LIBRARY_PATH EXCLUDE REGEX "anaconda|conda")
  list(FILTER CMAKE_INCLUDE_PATH EXCLUDE REGEX "anaconda|conda")
  # 明确设置系统 Protobuf 路径
  set(Protobuf_USE_STATIC_LIBS OFF)
  set(PROTOBUF_PROTOC_EXECUTABLE /usr/bin/protoc CACHE FILEPATH "Protobuf compiler")
  set(Protobuf_PROTOC_EXECUTABLE /usr/bin/protoc CACHE FILEPATH "Protobuf compiler")
endif()

# 优先使用系统库路径
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
list(INSERT CMAKE_PREFIX_PATH 0 "/usr" "/usr/local")

# =============================================================================
# 查找依赖包
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Protobuf - 使用系统版本（通过 FindProtobuf 模块）
# 不使用 CONFIG 模式，因为 Ubuntu 22.04 的 protobuf 包不提供 CMake config 文件
find_package(Protobuf REQUIRED)

# gRPC - 设为可选，避免 Anaconda 冲突
option(USE_GRPC "Enable gRPC support" OFF)
if(USE_GRPC)
  find_package(gRPC QUIET HINTS "/usr/lib/x86_64-linux-gnu/cmake/grpc")
  if(gRPC_FOUND)
    message(STATUS "gRPC found: ${gRPC_VERSION}")
    add_definitions(-DFSM_USE_GRPC)
  else()
    message(STATUS "gRPC not found, building without gRPC support")
  endif()
endif()

# nlohmann/json - 优先使用系统安装，否则使用本地 header-only 版本
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found in system, using bundled header-only version")
  # 使用项目内置的 header-only 版本
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party)
  # 创建一个接口库以保持兼容性
  add_library(nlohmann_json INTERFACE)
  target_include_directories(nlohmann_json INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
  add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
endif()

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# Autoware 消息包 (如果可用)
find_package(autoware_auto_vehicle_msgs QUIET)
find_package(autoware_auto_control_msgs QUIET)
find_package(autoware_auto_planning_msgs QUIET)
find_package(tier4_vehicle_msgs QUIET)

# WebRTC (libdatachannel)
find_package(LibDataChannel QUIET)

# Boost (for WebSocket server)
find_package(Boost REQUIRED COMPONENTS system)

# OpenSSL
find_package(OpenSSL REQUIRED)

# Threads
find_package(Threads REQUIRED)

# 设置包含目录
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle_node/include
  ${CMAKE_CURRENT_SOURCE_DIR}/cloud_server/include
  ${CMAKE_CURRENT_SOURCE_DIR}/operator_client/include
  ${OpenCV_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

# 编译 Proto 文件
set(PROTO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
file(GLOB PROTO_FILES "${PROTO_SRC_DIR}/*.proto")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# 添加 protobuf 生成文件的目录到包含路径
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# 公共库
add_library(fsm_common SHARED
  common/src/config_manager.cpp
  common/src/logger.cpp
  common/src/utils.cpp
  common/src/message_types.cpp
  ${PROTO_SRCS}
)
target_link_libraries(fsm_common
  yaml-cpp
  spdlog::spdlog
  ${PROTOBUF_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# 子项目
add_subdirectory(vehicle_node)
add_subdirectory(cloud_server)
add_subdirectory(operator_client)

# 安装
install(TARGETS fsm_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY common/include/
  DESTINATION include
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test)
    add_subdirectory(test)
  else()
    message(STATUS "Test directory not found, skipping tests")
  endif()
endif()

ament_package()
