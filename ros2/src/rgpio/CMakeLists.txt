cmake_minimum_required(VERSION 3.16)
project(rgpio)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(rutil REQUIRED)
find_package(rgpio_msgs REQUIRED)
find_package(rdata REQUIRED)

# rgpio executable
add_executable(rgpio_node src/main.cpp
                          lib/node.cpp
                          lib/real.cpp
                          lib/virtual.cpp
                          lib/helper.cpp)

target_include_directories(rgpio_node PRIVATE lib)

target_link_libraries(rgpio_node tomlplusplus::tomlplusplus
                                 gpiod)

ament_target_dependencies(rgpio_node rclcpp 
                                     rclcpp_lifecycle
                                     lifecycle_msgs
                                     rutil
                                     rgpio_msgs
                                     rdata)

install(DIRECTORY include/
        DESTINATION include)
                             
install(TARGETS rgpio_node
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)
                             
ament_package()
