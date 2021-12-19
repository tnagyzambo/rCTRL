cmake_minimum_required(VERSION 3.16)
project(rdata)

# Set compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Werror -Wextra -Wpedantic -Wvla -Wextra-semi -Wnull-dereference -Wswitch-enum")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include .cmake modules
include(cmake/Conan.cmake)

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           "rdata/msg/LogBool.msg"
                           "rdata/msg/LogF64.msg"
                           "rdata/msg/LogI64.msg"
                           "rdata/msg/LogStr.msg"
                           "rdata/msg/LogU64.msg"
                           "rdata/srv/CreateLogger.srv"
                           "rdata/srv/RemoveLogger.srv")

# Utilities library
add_library(rctl_util INTERFACE)

target_sources(rctl_util INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/util/Util.hpp)

target_include_directories(rctl_util INTERFACE util/) 

# rockerDATA interface library
add_library(rdata_iface INTERFACE)

target_sources(rdata_iface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/rdata_iface/RDataIFace.hpp)

target_include_directories(rdata_iface INTERFACE rdata_iface/)

# rocketDATA executable
add_executable(rdata_main rdata/src/main.cpp 
                          rdata/lib/RDataNode.cpp
                          rdata/lib/InfluxClient.cpp
                          rdata/lib/InfluxClientException.cpp)

target_include_directories(rdata_main PRIVATE rdata/lib)
target_link_libraries(rdata_main CONAN_PKG::libcurl) 
target_link_libraries(rdata_main CONAN_PKG::tomlplusplus)
target_link_libraries(rdata_main rdata_iface)
target_link_libraries(rdata_main rctl_util)

ament_target_dependencies(rdata_main rclcpp
                                     rclcpp_lifecycle
                                     lifecycle_msgs)
rosidl_target_interfaces(rdata_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rocketDATA Virtual Sensor executable
add_executable(rdata_vsensor rdata_vsensor/src/main.cpp
                             rdata_vsensor/lib/VSensorNode.cpp)

target_include_directories(rdata_vsensor PRIVATE rdata_vsensor/lib)
target_link_libraries(rdata_vsensor rdata_iface)
target_link_libraries(rdata_vsensor rctl_util)

ament_target_dependencies(rdata_vsensor rclcpp)
rosidl_target_interfaces(rdata_vsensor ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rocketDATA minimal example executable
add_executable(rdata_example rdata_example/src/main.cpp)

target_include_directories(rdata_example PRIVATE rdata_example/lib)
target_link_libraries(rdata_example rdata_iface)
target_link_libraries(rdata_example rctl_util)

ament_target_dependencies(rdata_example rclcpp
                                        rclcpp_lifecycle
                                        lifecycle_msgs)
rosidl_target_interfaces(rdata_example ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
        rdata_main
        rdata_vsensor
        rdata_example
        DESTINATION lib/${PROJECT_NAME})

ament_package()
