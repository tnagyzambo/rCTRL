cmake_minimum_required(VERSION 3.16)
project(rdata_vsensor)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rutil REQUIRED)
find_package(rdata REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

# rdata virtual sensor executable
add_executable(${PROJECT_NAME} src/main.cpp
                               src/vsensor_node.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE lib)

target_link_libraries(${PROJECT_NAME} fmt::fmt)

ament_target_dependencies(${PROJECT_NAME} rclcpp
                                          rclcpp_lifecycle
                                          lifecycle_msgs
                                          rutil
                                          rdata)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
