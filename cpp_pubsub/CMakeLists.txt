cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpp_custinterface REQUIRED)

# Add executables
add_executable(server src/publisher_member_function.cpp)
ament_target_dependencies(server
rclcpp cpp_custinterface)

add_executable(client src/subscriber_member_function.cpp)
ament_target_dependencies(client
rclcpp cpp_custinterface)

# install executables
install(TARGETS
  server  
  client
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()