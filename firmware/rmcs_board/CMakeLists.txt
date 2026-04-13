cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(rmcs_board_superbuild LANGUAGES NONE)

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/librmcs_firmware.cmake")

set(BOARD "pro" CACHE STRING "Target board (pro / lite)")
set_property(CACHE BOARD PROPERTY STRINGS pro lite)
set(BOARD_SEARCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/boards" CACHE PATH "Board search path")
set(RV_ARCH "rv32imafdcb" CACHE STRING "RISC-V arch")
set(RV_ABI "ilp32d" CACHE STRING "RISC-V ABI")
set(BOARD_BOOTLOADER_MODE "auto" CACHE STRING "Bootloader mode (auto / manual)")
set_property(CACHE BOARD_BOOTLOADER_MODE PROPERTY STRINGS auto manual)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "debug")
endif()

librmcs_setup_project_context(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

function(add_rmcs_board_subbuild target_name source_subdir)
    set(source_dir "${CMAKE_CURRENT_SOURCE_DIR}/${source_subdir}")
    set(binary_dir "${CMAKE_CURRENT_BINARY_DIR}/${source_subdir}")
    set(compile_commands_file "${binary_dir}/compile_commands.json")

    set(configure_args
        -G "${CMAKE_GENERATOR}"
        -S "${source_dir}"
        -B "${binary_dir}"
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DLIBRMCS_RMCS_BOARD_SUPERBUILDING=ON
        -UBOARD_SEARCH_DIR
        -USDK_BOARD_DIR
        -DBOARD=${BOARD}
        -DBOARD_SEARCH_PATH=${BOARD_SEARCH_PATH}
        -DRV_ARCH=${RV_ARCH}
        -DRV_ABI=${RV_ABI}
        -DHOST_DEBUGGER=${HOST_DEBUGGER}
        -DSOURCE_PATH_MAP=${LIBRMCS_SOURCE_PATH_MAP}
        -DLIBRMCS_PROJECT_VERSION=${LIBRMCS_PROJECT_VERSION}
    )
    if(CMAKE_MAKE_PROGRAM)
        list(APPEND configure_args -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM})
    endif()
    list(APPEND configure_args ${ARGN})

    add_custom_target(
        ${target_name}
        BYPRODUCTS "${compile_commands_file}"
        COMMAND ${CMAKE_COMMAND} ${configure_args}
        COMMAND ${CMAKE_COMMAND} --build "${binary_dir}"
        USES_TERMINAL
        VERBATIM
    )
endfunction()

add_rmcs_board_subbuild(rmcs_board_app app)
add_rmcs_board_subbuild(
    rmcs_board_bootloader
    bootloader
    -DBOARD_BOOTLOADER_MODE=${BOARD_BOOTLOADER_MODE}
)

set(RMCS_BOARD_APP_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/app/compile_commands.json")
set(RMCS_BOARD_BOOTLOADER_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/bootloader/compile_commands.json")
set(RMCS_BOARD_MERGED_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")

add_custom_command(
    OUTPUT "${RMCS_BOARD_MERGED_COMPILE_COMMANDS}"
    COMMAND
    ${CMAKE_COMMAND}
    "-DOUTPUT_FILE=${RMCS_BOARD_MERGED_COMPILE_COMMANDS}"
    "-DINPUT_FILES=${RMCS_BOARD_APP_COMPILE_COMMANDS};${RMCS_BOARD_BOOTLOADER_COMPILE_COMMANDS}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/merge_compile_commands.cmake"
    DEPENDS
    rmcs_board_app
    rmcs_board_bootloader
    "${RMCS_BOARD_APP_COMPILE_COMMANDS}"
    "${RMCS_BOARD_BOOTLOADER_COMPILE_COMMANDS}"
    COMMENT "Merging rmcs_board compile commands"
    VERBATIM
)

add_custom_target(
    rmcs_board_compile_commands
    DEPENDS "${RMCS_BOARD_MERGED_COMPILE_COMMANDS}"
)

add_custom_target(rmcs_board ALL DEPENDS rmcs_board_compile_commands)
