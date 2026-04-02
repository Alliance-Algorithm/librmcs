cmake_minimum_required(VERSION 3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(rmcs_board_superbuild LANGUAGES NONE)

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/librmcs_firmware.cmake")

set(BOARD "hpm5300evk" CACHE STRING "Target board")
set(RV_ARCH "rv32imafdcb" CACHE STRING "RISC-V arch")
set(RV_ABI "ilp32d" CACHE STRING "RISC-V ABI")
set(HPM_BUILD_TYPE "flash_xip" CACHE STRING "HPM build type")

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
        -DBOARD=${BOARD}
        -DRV_ARCH=${RV_ARCH}
        -DRV_ABI=${RV_ABI}
        -DHPM_BUILD_TYPE=${HPM_BUILD_TYPE}
        -DHOST_DEBUGGER=${HOST_DEBUGGER}
        -DSOURCE_PATH_MAP=${LIBRMCS_SOURCE_PATH_MAP}
        -DLIBRMCS_PROJECT_VERSION=${LIBRMCS_PROJECT_VERSION}
    )
    if(CMAKE_MAKE_PROGRAM)
        list(APPEND configure_args -DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM})
    endif()

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

set(RMCS_BOARD_APP_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/app/compile_commands.json")
set(RMCS_BOARD_COMPILE_COMMANDS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")

add_custom_command(
    OUTPUT "${RMCS_BOARD_COMPILE_COMMANDS}"
    COMMAND
    ${CMAKE_COMMAND}
    -E copy_if_different
    "${RMCS_BOARD_APP_COMPILE_COMMANDS}"
    "${RMCS_BOARD_COMPILE_COMMANDS}"
    DEPENDS
        rmcs_board_app
        "${RMCS_BOARD_APP_COMPILE_COMMANDS}"
    COMMENT "Copying rmcs_board compile commands"
    VERBATIM
)

add_custom_target(
    rmcs_board_compile_commands
    DEPENDS "${RMCS_BOARD_COMPILE_COMMANDS}"
)

add_custom_target(rmcs_board ALL DEPENDS rmcs_board_compile_commands)
