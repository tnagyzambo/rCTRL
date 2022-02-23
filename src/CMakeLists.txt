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
find_package(tomlplusplus REQUIRED)
find_package(CURL REQUIRED)

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           "rdata/iface/msg/LogBool.msg"
                           "rdata/iface/msg/LogF64.msg"
                           "rdata/iface/msg/LogI64.msg"
                           "rdata/iface/msg/LogStr.msg"
                           "rdata/iface/msg/LogU64.msg"
                           "rdata/iface/srv/CreateLogger.srv"
                           "rdata/iface/srv/RemoveLogger.srv")

# Utilities library
add_library(rctrl_util INTERFACE)

target_sources(rctrl_util INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} rctrl/util/lib/util.hpp)

target_include_directories(rctrl_util INTERFACE rctrl/util/lib) 

# rockerDATA interface library
add_library(rdata_iface INTERFACE)

target_sources(rdata_iface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} rdata/iface/lib/rdata_iface.hpp)

target_include_directories(rdata_iface INTERFACE rdata/iface/lib)

# rocketDATA executable
add_executable(rdata_main rdata/main/src/main.cpp 
                          rdata/main/lib/rdata_node.cpp
                          rdata/main/lib/influx_client.cpp
                          rdata/main/lib/influx_client_exception.cpp)

target_include_directories(rdata_main PRIVATE rdata/main/lib)
target_link_libraries(rdata_main CURL::libcurl) 
target_link_libraries(rdata_main tomlplusplus::tomlplusplus)
target_link_libraries(rdata_main rdata_iface)
target_link_libraries(rdata_main rctrl_util)

ament_target_dependencies(rdata_main rclcpp
                                     rclcpp_lifecycle
                                     lifecycle_msgs)
rosidl_target_interfaces(rdata_main ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rocketDATA Virtual Sensor executable
add_executable(rdata_vsensor rdata/example/vsensor/src/main.cpp
                             rdata/example/vsensor/lib/vsensor_node.cpp)

target_include_directories(rdata_vsensor PRIVATE rdata/example/vsensor/lib)
target_link_libraries(rdata_vsensor rdata_iface)
target_link_libraries(rdata_vsensor rctrl_util)

ament_target_dependencies(rdata_vsensor rclcpp)
rosidl_target_interfaces(rdata_vsensor ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rocketDATA minimal example executable
add_executable(rdata_example rdata/example/lc_client/src/main.cpp)

target_include_directories(rdata_example PRIVATE rdata/example/lc_client/lib)
target_link_libraries(rdata_example rdata_iface)
target_link_libraries(rdata_example rctrl_util)

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
