list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

# CMake file to handle Conan dependencies
# Get CMake wrapper for Conan
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/v0.17.0/conan.cmake"
                  "${CMAKE_BINARY_DIR}/conan.cmake"
                  TLS_VERIFY ON)
endif()

# Include the Conan CMake wrapper
include(${CMAKE_BINARY_DIR}/conan.cmake)

# Use Conan Cmake wrapper to setup dependencies
conan_cmake_configure(REQUIRES libcurl/7.79.1
                               tomlplusplus/2.5.0
                      GENERATORS cmake)

# Use default Conan settings
conan_cmake_autodetect(settings)

# Install using cmake as the generator
# Using cmake_find_package as the generator will break doxygen_add_docs() as the 
# Conan generated FindDoxygen.cmake elides the CMake provided FindDoxygen.cmake where
# the function is defined
# I'm not sure how to resolve this so for now Conan packages have to be references using the
# prefix CONAN_PKG::${PACKAGE_NAME}
conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    REMOTE conancenter
                    SETTINGS ${settings})

# This line makes the installed CMake packages visible to CMake
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# Surpress clang warnings on compilation of Conan libraries
# REFERENCE: https://github.com/bincrafters/community/issues/1100#issuecomment-604997132
set(CONAN_SYSTEM_INCLUDES ON)