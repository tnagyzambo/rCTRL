cmake_minimum_required(VERSION 3.16)
project(recu)

# Include .cmake modules
include(../cmake/compile_commands.cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(recu_msgs REQUIRED)
find_package(rutil REQUIRED)
find_package(rdata_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_executable(recu_node src/main.cpp
                         lib/node.cpp)

target_include_directories(recu_node PRIVATE lib)

target_link_libraries(recu_node nlohmann_json::nlohmann_json)

ament_target_dependencies(recu_node rclcpp
                                    rclcpp_lifecycle
                                    lifecycle_msgs
                                    recu_msgs
                                    rdata_msgs
                                    rutil)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS recu_node
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include)

ament_package()
