cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(APP_NAME "rmcs_board_bootloader")

if(NOT LIBRMCS_RMCS_BOARD_SUPERBUILDING)
    message(FATAL_ERROR
        "Do not configure firmware/rmcs_board/bootloader directly.\n"
        "Use the rmcs_board superbuild instead:\n")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake/librmcs_firmware.cmake")

set(HPM_BUILD_TYPE "flash_xip" CACHE STRING "HPM build type")
set(BOARD_BOOTLOADER_MODE "auto" CACHE STRING "Bootloader mode (auto / manual)")
set_property(CACHE BOARD_BOOTLOADER_MODE PROPERTY STRINGS auto manual)

set(CONFIG_TINYUSB 1)
set(CONFIG_USB_DEVICE 1)

if(NOT DEFINED ENV{HPM_SDK_BASE} OR "$ENV{HPM_SDK_BASE}" STREQUAL "")
    set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/hpm_sdk")
endif()
find_package(hpm-sdk REQUIRED HINTS "$ENV{HPM_SDK_BASE}")

librmcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

project(rmcs_board_bootloader VERSION 3 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

sdk_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)
sdk_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>)
if(LIBRMCS_SOURCE_PATH_MAP_COMPILE_OPTIONS)
    sdk_compile_options(${LIBRMCS_SOURCE_PATH_MAP_COMPILE_OPTIONS})
endif()

if(CMAKE_BUILD_TYPE STREQUAL "debug")
    sdk_compile_options(-Og)
endif()

if(NOT "${BOARD_BOOTLOADER_MODE}" STREQUAL "auto"
    AND NOT "${BOARD_BOOTLOADER_MODE}" STREQUAL "manual")
    message(FATAL_ERROR "BOARD_BOOTLOADER_MODE must be auto or manual")
endif()

if("${BOARD_BOOTLOADER_MODE}" STREQUAL "auto")
    set(bootloader_mode_auto 1)
else()
    set(bootloader_mode_auto 0)
endif()

sdk_compile_definitions(-DLIBRMCS_PROJECT_VERSION_STRING="${LIBRMCS_PROJECT_VERSION}")
sdk_compile_definitions(-DCFG_TUSB_MCU=OPT_MCU_HPM)
sdk_compile_definitions(-DUSB_HOST_MCU_CORE=HPM_CORE0)
sdk_compile_definitions(-DLIBRMCS_BOOTLOADER_MODE_AUTO=${bootloader_mode_auto})

sdk_inc("${CMAKE_CURRENT_SOURCE_DIR}/include")
sdk_app_inc("${LIBRMCS_PROJECT_ROOT}")

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
sdk_app_src(${PROJECT_SOURCE})
sdk_app_src(
    "$ENV{HPM_SDK_BASE}/middleware/tinyusb/src/device/usbd.c"
    "$ENV{HPM_SDK_BASE}/middleware/tinyusb/src/device/usbd_control.c"
    "$ENV{HPM_SDK_BASE}/middleware/tinyusb/src/portable/hpm/dcd_hpm.c"
    "$ENV{HPM_SDK_BASE}/middleware/tinyusb/src/class/dfu/dfu_device.c"
)

sdk_linker_global_symbols("_flash_size=0x1F000")

set_property(TARGET app PROPERTY CXX_STANDARD 23)
set_property(TARGET app PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET app PROPERTY CXX_EXTENSIONS OFF)

get_target_property(_hpm_incs ${HPM_SDK_LIB_ITF} INTERFACE_INCLUDE_DIRECTORIES)
if(_hpm_incs)
    set_target_properties(${HPM_SDK_LIB_ITF} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hpm_incs}")
endif()
