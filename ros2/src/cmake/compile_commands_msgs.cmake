# This has to be compatible with uros, there are some limitations here that are still
# not  fully understood
# REFERENCE: https://github.com/micro-ROS/micro_ros_arduino/issues/903

# Set compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
