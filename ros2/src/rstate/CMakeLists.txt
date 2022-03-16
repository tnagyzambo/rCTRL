cmake_minimum_required(VERSION 3.16)
project(rstate)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rutil REQUIRED)
find_package(rdata REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
                           "action/Transition.action")

# rstate executable
add_executable(rstate_node src/main.cpp
                           lib/cmd/service.cpp
                           lib/state/activating.cpp
                           lib/state/active.cpp
                           lib/state/armed.cpp
                           lib/state/arming.cpp
                           lib/state/cleaning_up.cpp
                           lib/state/configuring.cpp
                           lib/state/deactivating.cpp
                           lib/state/disarming.cpp
                           lib/state/error_processing.cpp
                           lib/state/finalized.cpp
                           lib/state/inactive.cpp
                           lib/state/shutting_down.cpp
                           lib/state/state.cpp
                           lib/state/unconfigured.cpp
                           lib/util/util.cpp
                           lib/node.cpp)

target_include_directories(rstate_node PRIVATE lib
                                               lib/cmd
                                               lib/state
                                               lib/util)

ament_target_dependencies(rstate_node rclcpp
                                      rclcpp_lifecycle
                                      rclcpp_action
                                      lifecycle_msgs
                                      rutil
                                      rdata)

rosidl_target_interfaces(rstate_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS rstate_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
