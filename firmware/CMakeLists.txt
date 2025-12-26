# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(BOARD "hpm5300evk" CACHE STRING "Target board")
set(RV_ARCH "rv32imafdcb" CACHE STRING "RISC-V arch")
set(RV_ABI "ilp32d" CACHE STRING "RISC-V ABI")
set(HPM_BUILD_TYPE "flash_xip" CACHE STRING "HPM build type")

set(CONFIG_TINYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_VENDOR 1)

if(NOT DEFINED ENV{HPM_SDK_BASE} OR "$ENV{HPM_SDK_BASE}" STREQUAL "")
    set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_LIST_DIR}/bsp/hpm_sdk")
endif()
find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(rmcs_slave LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)

set(APP_NAME rmcs_slave)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

sdk_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>)

option(HOST_DEBUGGER "Debugger runs outside Dev Container (Docker)" OFF)
set(SOURCE_PATH_MAP "" CACHE PATH "(In container) Map container source path to a host-visible path for debugger")

if(HOST_DEBUGGER AND SOURCE_PATH_MAP STREQUAL "")
    if(DEFINED ENV{HOST_WORKSPACE_FOLDER} AND NOT "$ENV{HOST_WORKSPACE_FOLDER}" STREQUAL "")
        set(SOURCE_PATH_MAP "$ENV{HOST_WORKSPACE_FOLDER}")
    else()
        message(FATAL_ERROR "HOST_WORKSPACE_FOLDER is not set")
    endif()
endif()
if(NOT SOURCE_PATH_MAP STREQUAL "")
    set(SOURCE_PATH_MAP_FROM "${CMAKE_CURRENT_SOURCE_DIR}/..")
    cmake_path(ABSOLUTE_PATH SOURCE_PATH_MAP_FROM NORMALIZE)

    set(SOURCE_PATH_MAP "${SOURCE_PATH_MAP}/")
    cmake_path(NORMAL_PATH SOURCE_PATH_MAP)

    sdk_compile_options(
        "-fdebug-prefix-map=${SOURCE_PATH_MAP_FROM}=${SOURCE_PATH_MAP}"
        "-ffile-prefix-map=${SOURCE_PATH_MAP_FROM}=${SOURCE_PATH_MAP}"
    )
endif()

# HPM SDK forcibly converts CMAKE_BUILD_TYPE to lowercase. God knows why.
if(CMAKE_BUILD_TYPE STREQUAL "debug")
    sdk_compile_options(-Og)
endif()

sdk_compile_definitions(-DCFG_TUSB_MCU=OPT_MCU_HPM)
sdk_compile_definitions(-DUSB_HOST_MCU_CORE=HPM_CORE0)

sdk_inc(${CMAKE_CURRENT_SOURCE_DIR}/include)

sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/..)

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/src/*.c
)
sdk_app_src(${PROJECT_SOURCE})

set_property(TARGET app PROPERTY CXX_STANDARD 23)
set_property(TARGET app PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET app PROPERTY CXX_EXTENSIONS OFF)

# Move SDK includes into SYSTEM to suppress third-party warnings
get_target_property(_hpm_incs ${HPM_SDK_LIB_ITF} INTERFACE_INCLUDE_DIRECTORIES)
if(_hpm_incs)
    set_target_properties(${HPM_SDK_LIB_ITF} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hpm_incs}")
endif()
