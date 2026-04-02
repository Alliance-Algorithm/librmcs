cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(APP_NAME "rmcs_board_bootloader")

include("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake/librmcs_firmware.cmake")

set(BOARD "hpm5300evk" CACHE STRING "Target board")
set(RV_ARCH "rv32imafdcb" CACHE STRING "RISC-V arch")
set(RV_ABI "ilp32d" CACHE STRING "RISC-V ABI")
set(HPM_BUILD_TYPE "flash_xip" CACHE STRING "HPM build type")

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

sdk_compile_definitions(-DLIBRMCS_PROJECT_VERSION_STRING="${LIBRMCS_PROJECT_VERSION}")
sdk_compile_definitions(-DCFG_TUSB_MCU=OPT_MCU_HPM)
sdk_compile_definitions(-DUSB_HOST_MCU_CORE=HPM_CORE0)

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
