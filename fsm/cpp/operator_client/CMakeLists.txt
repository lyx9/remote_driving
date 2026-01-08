# FSM-Pilot Operator Client CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# 操作端客户端库
add_library(fsm_operator_lib STATIC
  src/wheel_controller.cpp
  src/operator_client.cpp
)

target_include_directories(fsm_operator_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fsm_operator_lib PUBLIC
  fsm_common
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# 如果有 libdatachannel
if(LibDataChannel_FOUND)
  target_link_libraries(fsm_operator_lib PUBLIC
    LibDataChannel::LibDataChannel
  )
  target_compile_definitions(fsm_operator_lib PUBLIC HAS_LIBDATACHANNEL)
endif()

# 操作端可执行文件
add_executable(fsm_operator_client
  src/main.cpp
)

target_link_libraries(fsm_operator_client PRIVATE
  fsm_operator_lib
)

# 安装规则
install(TARGETS fsm_operator_client fsm_operator_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
