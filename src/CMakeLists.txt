cmake_minimum_required(VERSION 3.9)
project(rocketdata)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
                           "rocketdata/msg/LogBool.msg"
                           "rocketdata/msg/LogFloat64.msg"
                           "rocketdata/msg/LogInt64.msg"
                           "rocketdata/msg/LogString.msg"
                           "rocketdata/msg/LogUint64.msg")

# rocketDATA executable
add_executable(rocketdata_node rocketdata/src/main.cpp 
                               rocketdata/lib/RocketDataNode.cpp
                               rocketdata/lib/InfluxClient.cpp)

target_include_directories(rocketdata_node PRIVATE rocketdata/lib)
target_link_libraries(rocketdata_node CONAN_PKG::libcurl) 
target_link_libraries(rocketdata_node CONAN_PKG::tomlplusplus)

ament_target_dependencies(rocketdata_node rclcpp std_msgs)
rosidl_target_interfaces(rocketdata_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# RocketDATA Virtual Sensor executable
add_executable(rocketdata_virtual_sensor_node rocketdata_virtual_sensor/src/main.cpp
                                              rocketdata_virtual_sensor/lib/VirtualSensorNode.cpp)

target_include_directories(rocketdata_virtual_sensor_node PRIVATE rocketdata_virtual_sensor/lib)

ament_target_dependencies(rocketdata_virtual_sensor_node rclcpp std_msgs)
rosidl_target_interfaces(rocketdata_virtual_sensor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
        rocketdata_node
        rocketdata_virtual_sensor_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
