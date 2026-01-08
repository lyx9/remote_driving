# FSM-Pilot Cloud Server CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# 云端服务器库
add_library(fsm_cloud_lib STATIC
  src/scheduling_service.cpp
  src/signaling_server.cpp
  src/alert_analyzer.cpp
  src/prediction_interface.cpp
)

target_include_directories(fsm_cloud_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fsm_cloud_lib PUBLIC
  fsm_common
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  nlohmann_json::nlohmann_json
  OpenSSL::SSL
  OpenSSL::Crypto
  Threads::Threads
)

# 云端服务器可执行文件
add_executable(fsm_cloud_server
  src/main.cpp
)

target_link_libraries(fsm_cloud_server PRIVATE
  fsm_cloud_lib
)

# 安装规则
install(TARGETS fsm_cloud_server fsm_cloud_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/fsm_pilot/config
)
