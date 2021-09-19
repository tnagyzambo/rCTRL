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

add_executable(rocketdata src/rocketdata/main.cpp)
target_link_libraries(rocketdata CONAN_PKG::libcurl)
ament_target_dependencies(rocketdata rclcpp std_msgs)

add_executable(rocketdata_virtual_sensor src/rocketdata_virtual_sensor/main.cpp)
ament_target_dependencies(rocketdata_virtual_sensor rclcpp std_msgs)

install(TARGETS
        rocketdata
        rocketdata_virtual_sensor
        DESTINATION lib/${PROJECT_NAME})
