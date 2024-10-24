cmake_minimum_required(VERSION 3.12)
project(__librmcscpp)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(pybind11)
pybind11_add_module(__librmcscpp src/main.cpp)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(__librmcscpp
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
