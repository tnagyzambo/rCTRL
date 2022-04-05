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

include_directories(include)

# rgpio library
add_library(${PROJECT_NAME} SHARED src/input.cpp
                                   src/output.cpp
                                   src/gpio/real.cpp
                                   src/gpio/virtual.cpp
                                   src/util/util.cpp)

target_link_libraries(${PROJECT_NAME} tomlplusplus::tomlplusplus
                                      gpiod)

ament_target_dependencies(${PROJECT_NAME} rclcpp
                                          rgpio_msgs)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(tomlplusplus)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
