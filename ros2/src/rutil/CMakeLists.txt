cmake_minimum_required(VERSION 3.16)
project(rutil)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(fmt REQUIRED)

# REFERENCE: https://docs.ros.org/en/galactic/How-To-Guides/Ament-CMake-Documentation.html
# REFERENCE: https://github.com/ament/ament_index/blob/1df2aef4d7f1271d80082cc01aa08e13bfc58fce/ament_index_cpp/CMakeLists.txt

include_directories(include)

# rutil library
add_library(${PROJECT_NAME} SHARED src/except.cpp
                                   src/fmt.cpp
                                   src/toml.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC tomlplusplus::tomlplusplus
                                             fmt::fmt)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(tomlplusplus
                          fmt)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
