cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

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
find_package(tutorial_interfaces REQUIRED) 

add_executable(publisher_member_function src/publisher_member_function.cpp)
ament_target_dependencies(publisher_member_function rclcpp tutorial_interfaces)
add_executable(subscriber_member_function src/subscriber_member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp tutorial_interfaces)

install(TARGETS
  publisher_member_function
  subscriber_member_function
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()