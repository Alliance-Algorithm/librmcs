cmake_minimum_required(VERSION 3.10)

project(librmcs_example)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置 C++ 标准为 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置 C 标准为 C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

# 获取当前目录下的所有子目录
file(GLOB SUBDIRECTORIES RELATIVE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/*)

# 遍历每一个子目录
foreach(SUBDIR ${SUBDIRECTORIES})
    if(EXISTS "${CMAKE_SOURCE_DIR}/${SUBDIR}/main.cpp")
        # 添加可执行文件，名称为子目录名
        add_executable(${SUBDIR} ${CMAKE_SOURCE_DIR}/${SUBDIR}/main.cpp)
        # 为每个可执行文件链接libusb库
        target_link_libraries(${SUBDIR} -lusb-1.0)
    endif()
endforeach()

include_directories(${PROJECT_SOURCE_DIR}/..)