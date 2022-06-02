cmake_minimum_required(VERSION 3.16)
project(rdata)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rutil REQUIRED)
find_package(CURL REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

# rdata library
add_library(${PROJECT_NAME} SHARED src/logger.cpp
                                   src/except.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC tomlplusplus::tomlplusplus
                                             CURL::libcurl
                                             fmt::fmt
                                             rutil::rutil)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(tomlplusplus
                          CURL)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
