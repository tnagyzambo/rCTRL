cmake_minimum_required(VERSION 3.8)
project(test_pkg)

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

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
target_link_libraries(listener CONAN_PKG::libcurl)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
        talker
        listener
        DESTINATION lib/${PROJECT_NAME})
