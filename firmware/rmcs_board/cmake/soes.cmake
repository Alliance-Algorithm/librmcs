include(FetchContent)

# Integrate only the polling slave stack; the ESC HAL belongs to the app.
FetchContent_Declare(librmcs_soes
    URL https://codeload.github.com/OpenEtherCATsociety/SOES/tar.gz/6ef7b9415e936c061ad30626ca00004bdde2ac62
    URL_HASH SHA256=da67ea126244b92072f423631cc4c4d6891d1880bb70d065b24acf5230423ac6
    SOURCE_SUBDIR librmcs-unused
)
FetchContent_MakeAvailable(librmcs_soes)

set(soes_dir "${librmcs_soes_SOURCE_DIR}/soes")
target_include_directories(app SYSTEM PRIVATE "${soes_dir}" "${soes_dir}/include/sys/gcc")
target_include_directories(app PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/ecat")
sdk_app_src("${soes_dir}/ecat_slv.c" "${soes_dir}/esc.c" "${soes_dir}/esc_coe.c")
