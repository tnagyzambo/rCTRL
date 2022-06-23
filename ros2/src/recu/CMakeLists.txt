cmake_minimum_required(VERSION 3.16)
project(recu)

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
find_package(rdata REQUIRED)
find_package(rgpio REQUIRED)
find_package(rgpio_msgs REQUIRED)
find_package(recu_msgs REQUIRED)
find_package(ri2c_msgs REQUIRED)

# rdata executable
add_executable(${PROJECT_NAME} src/main.cpp 
                               lib/node.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE lib)

target_link_libraries(${PROJECT_NAME} tomlplusplus::tomlplusplus)

ament_target_dependencies(${PROJECT_NAME} rclcpp
                                          rclcpp_lifecycle
                                          lifecycle_msgs
                                          rutil
                                          rdata
                                          rgpio
                                          rgpio_msgs
                                          recu_msgs
                                          ri2c_msgs)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION)

ament_package()
