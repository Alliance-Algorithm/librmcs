cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME librmcs-firmware)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
enable_language(C CXX ASM)

set(LIBRMCS_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
cmake_path(ABSOLUTE_PATH LIBRMCS_PROJECT_ROOT NORMALIZE)

set(LIBRMCS_TINYUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/tinyusb")
if(NOT EXISTS "${LIBRMCS_TINYUSB_ROOT}/src/tusb.h")
    message(FATAL_ERROR "TinyUSB submodule not found at ${LIBRMCS_TINYUSB_ROOT}. "
        "Run: git submodule update --init firmware/c_board/bsp/tinyusb")
endif()

if(NOT DEFINED LIBRMCS_PROJECT_VERSION OR LIBRMCS_PROJECT_VERSION STREQUAL "")
    execute_process(
        COMMAND ".scripts/generate_version"
        WORKING_DIRECTORY "${LIBRMCS_PROJECT_ROOT}"
        OUTPUT_VARIABLE LIBRMCS_PROJECT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        COMMAND_ERROR_IS_FATAL ANY
    )
endif()
message(STATUS "Librmcs project version: ${LIBRMCS_PROJECT_VERSION}")

add_executable(${CMAKE_PROJECT_NAME})

# Add libstdc++ for C++ code. This is consumed by stm32cubemx/CMakeLists.txt
# via TOOLCHAIN_LINK_LIBRARIES.
list(APPEND TOOLCHAIN_LINK_LIBRARIES "stdc++")

# Bring in CubeMX generated HAL build definitions.
add_subdirectory(bsp/HAL/cmake/stm32cubemx)

add_library(tinyusb_device OBJECT)
target_sources(tinyusb_device PRIVATE
    "${LIBRMCS_TINYUSB_ROOT}/src/tusb.c"
    "${LIBRMCS_TINYUSB_ROOT}/src/common/tusb_fifo.c"
    "${LIBRMCS_TINYUSB_ROOT}/src/device/usbd.c"
    "${LIBRMCS_TINYUSB_ROOT}/src/device/usbd_control.c"
    "${LIBRMCS_TINYUSB_ROOT}/src/class/vendor/vendor_device.c"
    "${LIBRMCS_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c"
    "${LIBRMCS_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c"
)
target_include_directories(tinyusb_device SYSTEM PUBLIC
    "${LIBRMCS_TINYUSB_ROOT}/src"
)
target_include_directories(tinyusb_device PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_definitions(tinyusb_device PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32F4
)
target_link_libraries(tinyusb_device PUBLIC
    stm32cubemx
)

# Mark CubeMX/HAL headers as SYSTEM so they show up as `-isystem` in
# compile_commands.json (clang-tidy should not warn on third-party code).
if(TARGET stm32cubemx)
    get_target_property(HAL_INCLUDE_DIRS stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
    if(HAL_INCLUDE_DIRS AND NOT HAL_INCLUDE_DIRS STREQUAL "HAL_INCLUDE_DIRS-NOTFOUND")
        set_property(TARGET stm32cubemx PROPERTY INTERFACE_INCLUDE_DIRECTORIES "")
        target_include_directories(stm32cubemx SYSTEM INTERFACE ${HAL_INCLUDE_DIRS})
    endif()
endif()

# --- Compile options ---
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -fvisibility=hidden
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# --- HOST_DEBUGGER support ---
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
    set(SOURCE_PATH_MAP "${SOURCE_PATH_MAP}/")
    cmake_path(NORMAL_PATH SOURCE_PATH_MAP)

    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        "-fdebug-prefix-map=${LIBRMCS_PROJECT_ROOT}=${SOURCE_PATH_MAP}"
        "-ffile-prefix-map=${LIBRMCS_PROJECT_ROOT}=${SOURCE_PATH_MAP}"
    )
endif()

# Debug build: append -Og (overrides toolchain's -O0, GCC takes last)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Og)
endif()

# --- User application sources ---
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS src/*.cpp src/*.c)
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    "${LIBRMCS_PROJECT_ROOT}/core/src/*.cpp"
    "${LIBRMCS_PROJECT_ROOT}/core/src/*.c"
)
file(GLOB SEGGER_C_SOURCES bsp/SEGGER/RTT/*.c)
file(GLOB SEGGER_ASM_SOURCES bsp/SEGGER/RTT/*.S)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
    ${CORE_SOURCES}
    ${SEGGER_C_SOURCES}
    ${SEGGER_ASM_SOURCES}
)

# --- Include paths ---
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${LIBRMCS_PROJECT_ROOT}
)

# SEGGER RTT is third-party code; treat its headers as system headers.
target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/SEGGER/RTT
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/SEGGER/Config
)

# --- Defines ---
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    LIBRMCS_PROJECT_VERSION_STRING="${LIBRMCS_PROJECT_VERSION}"
    $<$<CONFIG:Release>:NDEBUG>
)

# Remove wrong libob.a dependency (CubeMX toolchain quirk)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    tinyusb_device
)
