cmake_minimum_required(VERSION 3.16)
project(rdata)

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

include_directories(include)

# Create ros2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
                           "msg/LogBool.msg"
                           "msg/LogF64.msg"
                           "msg/LogI64.msg"
                           "msg/LogStr.msg"
                           "msg/LogU64.msg"
                           "srv/CreateLogger.srv"
                           "srv/RemoveLogger.srv")

# rdata executable
add_executable(rdata_node src/main.cpp 
                          lib/rdata_node.cpp
                          lib/influx_client.cpp
                          lib/influx_exception.cpp)

target_include_directories(rdata_node PRIVATE lib)

target_link_libraries(rdata_node CURL::libcurl
                                 tomlplusplus::tomlplusplus)

ament_target_dependencies(rdata_node rclcpp
                                     rclcpp_lifecycle
                                     lifecycle_msgs
                                     rutil)

rosidl_target_interfaces(rdata_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rdata virtual sensor executable
add_executable(rdata_vsensor vsensor/src/main.cpp
                             vsensor/lib/vsensor_node.cpp)

target_include_directories(rdata_vsensor PRIVATE vsensor/lib)

ament_target_dependencies(rdata_vsensor rclcpp
                                        rclcpp_lifecycle
                                        lifecycle_msgs
                                        rutil)

rosidl_target_interfaces(rdata_vsensor ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS rdata_node rdata_vsensor
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

ament_package()
