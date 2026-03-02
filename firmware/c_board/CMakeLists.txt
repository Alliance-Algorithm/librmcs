cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


project(librmcs-firmware LANGUAGES C CXX ASM)

set(LIBRMCS_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
cmake_path(ABSOLUTE_PATH LIBRMCS_PROJECT_ROOT NORMALIZE)


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
    add_compile_options(
        "-fdebug-prefix-map=${LIBRMCS_PROJECT_ROOT}=${SOURCE_PATH_MAP}"
        "-ffile-prefix-map=${LIBRMCS_PROJECT_ROOT}=${SOURCE_PATH_MAP}"
    )
endif()


set(LIBRMCS_STM32_HAL_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/stm32f4xx-hal-driver")
set(LIBRMCS_STM32_DEVICE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/cmsis-device-f4")
set(LIBRMCS_CMSIS_CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/cmsis-core")
set(LIBRMCS_STM32_CUBEMX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/cubemx")

add_subdirectory("${LIBRMCS_STM32_CUBEMX_ROOT}/cmake/stm32cubemx" SYSTEM)


function(c_board_apply_target_options target linker_file)
    target_include_directories(${target} PRIVATE
        ${LIBRMCS_PROJECT_ROOT}
    )

    target_compile_options(${target} PRIVATE
        -fvisibility=hidden
        $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    )

    target_compile_definitions(${target} PRIVATE
        LIBRMCS_PROJECT_VERSION_STRING="${LIBRMCS_PROJECT_VERSION}"
        $<$<CONFIG:Release>:NDEBUG>
    )

    target_link_libraries(${target} PRIVATE
        stdc++
        m
        stm32cubemx
    )
    target_link_options(${target} PRIVATE
        "-Wl,-Map=$<TARGET_FILE_DIR:${target}>/$<TARGET_FILE_BASE_NAME:${target}>.map"
        "-T${LIBRMCS_STM32_CUBEMX_ROOT}/${linker_file}"
    )
endfunction()


add_subdirectory(app)
