cmake_minimum_required(VERSION 3.16)
project(rstate_msgs)

# Include .cmake modules
include(../cmake/compile_commands_msgs.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
   "msg/NetworkState.msg"
   "msg/NetworkTransition.msg"
   "msg/NetworkTransitionDescription.msg"
   "msg/NetworkTransitionEvent.msg"
   "msg/NetworkTransitionFeedback.msg")

set(srv_files
    "srv/GetAvailableNetworkStates.srv"
    "srv/GetAvailableNetworkTransitions.srv"
    "srv/GetNetworkState.srv"
    "srv/NetworkTransitionCancelGoal.srv"
    "srv/NetworkTransitionSendGoal.srv")

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           ${msg_files}
                           ${srv_files})

ament_package()
