cmake_minimum_required(VERSION 3.16)
project(rstate)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rutil REQUIRED)
find_package(rdata REQUIRED)

# rstate executable
add_executable(rstate_node src/main.cpp)

ament_target_dependencies(rstate_node rclcpp
                                      rclcpp_lifecycle
                                      lifecycle_msgs
                                      rutil
                                      rdata)

install(TARGETS rstate_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
