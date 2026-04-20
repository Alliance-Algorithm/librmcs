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

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/librmcs_firmware.cmake")
librmcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

if(LIBRMCS_SOURCE_PATH_MAP_COMPILE_OPTIONS)
    add_compile_options(${LIBRMCS_SOURCE_PATH_MAP_COMPILE_OPTIONS})
endif()


set(LIBRMCS_STM32_HAL_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/stm32f4xx-hal-driver")
set(LIBRMCS_STM32_DEVICE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/cmsis-device-f4")
set(LIBRMCS_CMSIS_CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/cmsis-core")
set(LIBRMCS_STM32_CUBEMX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/bsp/cubemx")

add_subdirectory("${LIBRMCS_STM32_CUBEMX_ROOT}/cmake/stm32cubemx" SYSTEM)


function(c_board_apply_target_options target linker_file)
    target_include_directories(${target} PRIVATE
        ${LIBRMCS_PROJECT_ROOT}/core/include
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
add_subdirectory(bootloader)
