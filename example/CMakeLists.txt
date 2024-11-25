cmake_minimum_required(VERSION 3.15)

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
    add_compile_options(/W4 /Zc:preprocessor)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    # GCC/Clang
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set libusb binaries directory (if provided)
set(LIBUSB_BINARY_PATH "" CACHE STRING "Path to libusb binaries directory")
if (NOT LIBUSB_BINARY_PATH STREQUAL "")
    message(STATUS "Using libusb binary from: ${LIBUSB_BINARY_PATH}")
    link_directories(${LIBUSB_BINARY_PATH})
endif()

# Set libusb header files directory (if provided)
if (UNIX)
    set(LIBUSB_HEADER_PATH "/usr/include/libusb-1.0" CACHE STRING "Path to libusb header files directory")
else()
    set(LIBUSB_HEADER_PATH "" CACHE STRING "Path to libusb header files directory")
endif()
if (NOT LIBUSB_HEADER_PATH STREQUAL "")
    message(STATUS "Using libusb header from: ${LIBUSB_HEADER_PATH}")
    include_directories(SYSTEM ${LIBUSB_HEADER_PATH})
endif()

# Get all subdirectories
file(GLOB SUBDIRECTORIES RELATIVE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/*)

# Iterate over each subdirectory
foreach(SUBDIR ${SUBDIRECTORIES})
    if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/${SUBDIR}" AND EXISTS "${CMAKE_SOURCE_DIR}/${SUBDIR}/main.cpp")
        # Add executable named after the subdirectory
        add_executable(${SUBDIR} ${CMAKE_SOURCE_DIR}/${SUBDIR}/main.cpp)
        
        # Link libusb library
        if(WIN32)
            target_link_libraries(${SUBDIR} PRIVATE libusb-1.0)
        elseif(UNIX)
            target_link_libraries(${SUBDIR} PRIVATE usb-1.0)
        endif()
    endif()
endforeach()

include_directories(${PROJECT_SOURCE_DIR}/..)
