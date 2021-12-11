cmake_minimum_required(VERSION 3.9)
project(rdata)

# Set compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Werror -Wextra -Wpedantic -Wvla -Wextra-semi -Wnull-dereference -Wswitch-enum")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Include .cmake modules
include(cmake/Conan.cmake)

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           "rdata/msg/LogBool.msg"
                           "rdata/msg/LogF64.msg"
                           "rdata/msg/LogI64.msg"
                           "rdata/msg/LogStr.msg"
                           "rdata/msg/LogU64.msg"
                           "rdata/srv/CreateSub.srv"
                           "rdata/srv/RemoveSub.srv")

# rocketDATA executable
add_executable(rdata_node rdata/src/main.cpp 
                               rdata/lib/RDataNode.cpp
                               rdata/lib/InfluxClient.cpp
                               rdata/lib/InfluxClientException.cpp)

target_include_directories(rdata_node PRIVATE rdata/lib)
target_link_libraries(rdata_node CONAN_PKG::libcurl) 
target_link_libraries(rdata_node CONAN_PKG::tomlplusplus)

ament_target_dependencies(rdata_node rclcpp std_msgs)
rosidl_target_interfaces(rdata_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# RocketDATA Virtual Sensor executable
add_executable(rocketdata_virtual_sensor_node rocketdata_virtual_sensor/src/main.cpp
                                              rocketdata_virtual_sensor/lib/VirtualSensorNode.cpp)

target_include_directories(rocketdata_virtual_sensor_node PRIVATE rocketdata_virtual_sensor/lib)

ament_target_dependencies(rocketdata_virtual_sensor_node rclcpp std_msgs)
rosidl_target_interfaces(rocketdata_virtual_sensor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
        rdata_node
        rocketdata_virtual_sensor_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
