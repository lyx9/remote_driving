cmake_minimum_required(VERSION 3.16)
project(fsm_vehicle_node)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Autoware 消息 (可选)
find_package(autoware_auto_vehicle_msgs QUIET)
find_package(autoware_auto_control_msgs QUIET)
find_package(tier4_vehicle_msgs QUIET)

# 设置源文件
set(VEHICLE_NODE_SOURCES
    src/vehicle_node.cpp
    src/data_collector.cpp
    src/video_encoder.cpp
    src/command_executor.cpp
    src/webrtc_client.cpp
    src/latency_monitor.cpp
)

# 创建可执行文件
add_executable(fsm_vehicle_node ${VEHICLE_NODE_SOURCES})

target_include_directories(fsm_vehicle_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(fsm_vehicle_node
    fsm_common
    ${OpenCV_LIBRARIES}
)

# ROS2 依赖
ament_target_dependencies(fsm_vehicle_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    image_transport
)

# 如果 Autoware 消息可用
if(autoware_auto_vehicle_msgs_FOUND)
    ament_target_dependencies(fsm_vehicle_node autoware_auto_vehicle_msgs)
    target_compile_definitions(fsm_vehicle_node PRIVATE HAVE_AUTOWARE_MSGS)
endif()

if(autoware_auto_control_msgs_FOUND)
    ament_target_dependencies(fsm_vehicle_node autoware_auto_control_msgs)
endif()

if(tier4_vehicle_msgs_FOUND)
    ament_target_dependencies(fsm_vehicle_node tier4_vehicle_msgs)
endif()

# 安装
install(TARGETS fsm_vehicle_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
