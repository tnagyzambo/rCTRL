cmake_minimum_required(VERSION 3.16)
project(ri2c)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# rgpio library
# add_library(${PROJECT_NAME} SHARED src/test.cpp)

# target_link_libraries(${PROJECT_NAME} tomlplusplus::tomlplusplus
#                                       gpiod)

# ament_target_dependencies(${PROJECT_NAME} rclcpp
#                                           rutil)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(tomlplusplus)

# install(DIRECTORY include/
#         DESTINATION include)

# install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION lib/${PROJECT_NAME}
#         INCLUDES DESTINATION include)

# ament_package()

add_executable(ri2c_node src/main.cpp)

target_include_directories(ri2c_node PRIVATE lib)

target_link_libraries(ri2c_node i2c)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS ri2c_node
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
