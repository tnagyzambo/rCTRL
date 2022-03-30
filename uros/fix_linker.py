# There are issues using platformio link against the static H7 uros library
# REFERENCE: https://github.com/micro-ROS/micro_ros_arduino/issues/774

Import("env")
env["_LIBFLAGS"] =  ('-Wl,--start-group -Wl,--whole-archive '
                    '${_stripixes(LIBLINKPREFIX, LIBS, LIBLINKSUFFIX, LIBPREFIXES, '
                    'LIBSUFFIXES, __env__)} -Wl,--no-whole-archive -lstdc++ '
                    '-lsupc++ -lm -lc -lgcc -lnosys -lmicroros -Wl,--end-group')
