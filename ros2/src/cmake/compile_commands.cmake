set(CMAKE_C_COMPILER /bin/clang-14)
set(CMAKE_CXX_COMPILER /bin/clang++-14)

# Set compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Werror -Wextra -Wpedantic -Wvla -Wextra-semi -Wnull-dereference -Wswitch-enum")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
