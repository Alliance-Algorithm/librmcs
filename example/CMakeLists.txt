cmake_minimum_required(VERSION 3.10)

project(librmcs_example)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set C standard to C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Disable GNU extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)

# Set default build type to Release With Debug Info
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Add compiler options based on compiler
if(MSVC)
    add_compile_options(/W4)  # MSVC
else()
    add_compile_options(-Wall -Wextra -Wpedantic)  # GCC/Clang
endif()

# Get all subdirectories
file(GLOB SUBDIRECTORIES RELATIVE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/*)

# Iterate over each subdirectory
foreach(SUBDIR ${SUBDIRECTORIES})
    if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/${SUBDIR}" AND EXISTS "${CMAKE_SOURCE_DIR}/${SUBDIR}/main.cpp")
        # Add executable named after the subdirectory
        add_executable(${SUBDIR} ${CMAKE_SOURCE_DIR}/${SUBDIR}/main.cpp)
        
        # Link libusb library
        target_link_libraries(${SUBDIR} PRIVATE usb-1.0)
    endif()
endforeach()

include_directories(${PROJECT_SOURCE_DIR}/..)

# Disable the cache of build type and rely on either the default or the provided one
unset(CMAKE_BUILD_TYPE CACHE)
