cmake_minimum_required(VERSION 3.8)
project(rocketdata)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include .cmake modules
include(cmake/Conan.cmake)

ament_package()

add_library(rocketdata_lib STATIC rocketdata/lib/RosNode.cpp
                                  rocketdata/lib/InfluxClient.cpp)
target_link_libraries(rocketdata_lib CONAN_PKG::libcurl)
target_link_libraries(rocketdata_lib CONAN_PKG::tomlplusplus)
ament_target_dependencies(rocketdata_lib rclcpp std_msgs)

add_executable(rocketdata rocketdata/src/main.cpp)
target_link_libraries(rocketdata rocketdata_lib)

add_executable(rocketdata_virtual_sensor rocketdata_virtual_sensor/src/main.cpp)
ament_target_dependencies(rocketdata_virtual_sensor rclcpp std_msgs)

install(TARGETS
        rocketdata
        rocketdata_virtual_sensor
        DESTINATION lib/${PROJECT_NAME})
