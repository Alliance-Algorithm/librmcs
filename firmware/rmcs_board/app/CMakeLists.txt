cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(APP_NAME "rmcs_board_app")

if(NOT LIBRMCS_RMCS_BOARD_SUPERBUILDING)
    message(FATAL_ERROR
        "Do not configure firmware/rmcs_board/app directly.\n"
        "Use the rmcs_board superbuild instead:\n")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake/librmcs_firmware.cmake")

set(HPM_BUILD_TYPE "flash_uf2" CACHE STRING "HPM build type")

set(CONFIG_DMA_MGR 1)
set(CONFIG_TINYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_VENDOR 1)
set(LIBRMCS_BUILDING_FIRMWARE_APP 1)

if(NOT DEFINED ENV{HPM_SDK_BASE} OR "$ENV{HPM_SDK_BASE}" STREQUAL "")
    set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/hpm_sdk")
endif()
find_package(hpm-sdk REQUIRED HINTS "$ENV{HPM_SDK_BASE}")

get_property(LIBRMCS_USB_PID GLOBAL PROPERTY LIBRMCS_USB_PID)
if("${LIBRMCS_USB_PID}" STREQUAL "")
    message(FATAL_ERROR "LIBRMCS_USB_PID must be set by the selected board")
endif()

librmcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

project(rmcs_board_app VERSION 3 LANGUAGES C CXX ASM)

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
    "${LIBRMCS_PROJECT_ROOT}/core/src/*.cpp"
    "${LIBRMCS_PROJECT_ROOT}/core/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
sdk_app_src(${PROJECT_SOURCE})
sdk_app_src("$ENV{HPM_SDK_BASE}/middleware/tinyusb/src/class/dfu/dfu_rt_device.c")

sdk_linker_global_symbols("_uf2_bl_length=0x20000")

set_property(TARGET app PROPERTY CXX_STANDARD 23)
set_property(TARGET app PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET app PROPERTY CXX_EXTENSIONS OFF)

get_target_property(_hpm_incs ${HPM_SDK_LIB_ITF} INTERFACE_INCLUDE_DIRECTORIES)
if(_hpm_incs)
    set_target_properties(${HPM_SDK_LIB_ITF} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_hpm_incs}")
endif()

librmcs_add_dfu_image(
    TARGET ${APP_ELF_NAME}
    INPUT_BINARY "${EXECUTABLE_OUTPUT_PATH}/${APP_BIN_NAME}"
    OUTPUT_DFU "${EXECUTABLE_OUTPUT_PATH}/rmcs_board_app.dfu"
    VENDOR_ID 0xA11C
    PRODUCT_ID ${LIBRMCS_USB_PID}
    DEVICE_ID 0x0300
    COMMENT "Generating rmcs_board_app.dfu"
)
