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
find_package(rstate_msgs REQUIRED)

# rstate executable
add_executable(${PROJECT_NAME} src/main.cpp
                               lib/cmd/service.cpp
                               lib/state/activating.cpp
                               lib/state/active.cpp
                               lib/state/cleaning_up.cpp
                               lib/state/configuring.cpp
                               lib/state/deactivating.cpp
                               lib/state/error_processing.cpp
                               lib/state/finalized.cpp
                               lib/state/inactive.cpp
                               lib/state/shutting_down.cpp
                               lib/state/state.cpp
                               lib/state/unconfigured.cpp
                               lib/state/unknown.cpp
                               lib/util/util.cpp
                               lib/node.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE lib
                                                   lib/cmd
                                                   lib/state
                                                   lib/util)

ament_target_dependencies(${PROJECT_NAME} rclcpp
                                          rclcpp_lifecycle
                                          rclcpp_action
                                          lifecycle_msgs
                                          rutil
                                          rstate_msgs)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION)
                                  
ament_package()
