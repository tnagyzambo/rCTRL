cmake_minimum_required(VERSION 3.16)
project(ri2c_msgs)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/ValveState.msg")

set(srv_files
    "srv/HighSpeedDataLoggingAction.srv"
    "srv/PresControlLoopAction.srv"
    "srv/GetValveState.srv")

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           ${msg_files}
                           ${srv_files}
                           ADD_LINTER_TESTS)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
