cmake_minimum_required(VERSION 3.16)
project(rgpio_example)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CURL REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(rutil REQUIRED)
find_package(rgpio REQUIRED)
find_package(rgpio_msgs REQUIRED)

include_directories(include)

# rdata executable
add_executable(rgpio_example_node src/main.cpp 
                                  lib/node.cpp)

target_include_directories(rgpio_example_node PRIVATE lib)

target_link_libraries(rgpio_example_node tomlplusplus::tomlplusplus)

ament_target_dependencies(rgpio_example_node rclcpp
                                             rclcpp_lifecycle
                                             lifecycle_msgs
                                             rutil
                                             rgpio
                                             rgpio_msgs)

install(TARGETS rgpio_example_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
