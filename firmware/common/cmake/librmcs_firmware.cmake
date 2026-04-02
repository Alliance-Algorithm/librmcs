include_guard(GLOBAL)

function(librmcs_setup_project_context)
    set(options)
    set(one_value_args PROJECT_ROOT)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "" ${ARGN})

    if(NOT ARG_PROJECT_ROOT)
        message(FATAL_ERROR "librmcs_setup_project_context requires PROJECT_ROOT")
    endif()

    set(project_root "${ARG_PROJECT_ROOT}")
    cmake_path(ABSOLUTE_PATH project_root NORMALIZE)

    option(HOST_DEBUGGER "Debugger runs outside Dev Container (Docker)" OFF)
    set(
        SOURCE_PATH_MAP
        ""
        CACHE PATH
        "(In container) Map container source path to a host-visible path for debugger"
    )

    set(project_version "${LIBRMCS_PROJECT_VERSION}")
    if("${project_version}" STREQUAL "")
        execute_process(
            COMMAND ".scripts/generate_version"
            WORKING_DIRECTORY "${project_root}"
            OUTPUT_VARIABLE project_version
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ERROR_IS_FATAL ANY
        )
    endif()

    set(source_path_map "${SOURCE_PATH_MAP}")
    if(HOST_DEBUGGER AND "${source_path_map}" STREQUAL "")
        if(DEFINED ENV{HOST_WORKSPACE_FOLDER} AND NOT "$ENV{HOST_WORKSPACE_FOLDER}" STREQUAL "")
            set(source_path_map "$ENV{HOST_WORKSPACE_FOLDER}")
        else()
            message(FATAL_ERROR "HOST_WORKSPACE_FOLDER is not set")
        endif()
    endif()

    set(prefix_map_compile_options "")
    if(NOT "${source_path_map}" STREQUAL "")
        set(source_path_map "${source_path_map}/")
        cmake_path(NORMAL_PATH source_path_map)
        list(
            APPEND
            prefix_map_compile_options
            "-fdebug-prefix-map=${project_root}=${source_path_map}"
            "-ffile-prefix-map=${project_root}=${source_path_map}"
        )
    endif()

    message(STATUS "Librmcs project version: ${project_version}")

    set(LIBRMCS_PROJECT_ROOT "${project_root}" PARENT_SCOPE)
    set(LIBRMCS_PROJECT_VERSION "${project_version}" PARENT_SCOPE)
    set(LIBRMCS_SOURCE_PATH_MAP "${source_path_map}" PARENT_SCOPE)
    set(LIBRMCS_SOURCE_PATH_MAP_COMPILE_OPTIONS "${prefix_map_compile_options}" PARENT_SCOPE)
endfunction()

function(librmcs_add_dfu_image)
    if(NOT DEFINED LIBRMCS_PROJECT_ROOT OR "${LIBRMCS_PROJECT_ROOT}" STREQUAL "")
        message(FATAL_ERROR "LIBRMCS_PROJECT_ROOT must be set before calling librmcs_add_dfu_image")
    endif()

    set(options)
    set(
        one_value_args
        TARGET
        INPUT_BINARY
        INPUT_ELF
        OUTPUT_BINARY
        OUTPUT_DFU
        VENDOR_ID
        PRODUCT_ID
        DEVICE_ID
        COMMENT
    )
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "librmcs_add_dfu_image requires TARGET")
    endif()
    if(NOT ARG_OUTPUT_DFU)
        message(FATAL_ERROR "librmcs_add_dfu_image requires OUTPUT_DFU")
    endif()

    set(input_binary "${ARG_INPUT_BINARY}")
    set(objcopy_command)
    if(NOT "${ARG_OUTPUT_BINARY}" STREQUAL "")
        if("${ARG_INPUT_ELF}" STREQUAL "")
            message(FATAL_ERROR "OUTPUT_BINARY requires INPUT_ELF")
        endif()
        if(NOT DEFINED CMAKE_OBJCOPY OR "${CMAKE_OBJCOPY}" STREQUAL "")
            message(FATAL_ERROR "CMAKE_OBJCOPY must be set when OUTPUT_BINARY is used")
        endif()

        set(input_binary "${ARG_OUTPUT_BINARY}")
        set(
            objcopy_command
            COMMAND
            ${CMAKE_OBJCOPY}
            -O
            binary
            ${ARG_INPUT_ELF}
            ${ARG_OUTPUT_BINARY}
        )
    endif()

    if("${input_binary}" STREQUAL "")
        message(FATAL_ERROR "librmcs_add_dfu_image requires INPUT_BINARY or OUTPUT_BINARY")
    endif()

    add_custom_command(
        TARGET ${ARG_TARGET}
        POST_BUILD
        ${objcopy_command}
        COMMAND
        "${LIBRMCS_PROJECT_ROOT}/.scripts/append_image_hash"
        -o
        ${ARG_OUTPUT_DFU}
        ${input_binary}
        COMMAND
        dfu-suffix
        -v
        ${ARG_VENDOR_ID}
        -p
        ${ARG_PRODUCT_ID}
        -d
        ${ARG_DEVICE_ID}
        -a
        ${ARG_OUTPUT_DFU}
        COMMENT "${ARG_COMMENT}"
        VERBATIM
    )
endfunction()
