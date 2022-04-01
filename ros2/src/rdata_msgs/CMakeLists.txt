cmake_minimum_required(VERSION 3.16)
project(rdata_msgs)

# Include .cmake modules
include(../cmake/compile_commands_msgs.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/LogBool.msg"
    "msg/LogF64.msg"
    "msg/LogI64.msg"
    "msg/LogStr.msg"
    "msg/LogU64.msg")

set(srv_files
    "srv/CreateLogger.srv"
    "srv/RemoveLogger.srv")

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           ${msg_files}
                           ${srv_files})

ament_package()
