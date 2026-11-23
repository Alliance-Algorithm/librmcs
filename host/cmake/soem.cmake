include(FetchContent)

FetchContent_Declare(librmcs_soem
    URL https://codeload.github.com/OpenEtherCATsociety/SOEM/tar.gz/abbf0d42e38d6cfbaa4c1e9e8e07ace651c386fd
    URL_HASH SHA256=17f35473343fb06e9a2838d78cd75dac78c95b834a10ec4141c5881aec899aeb
    SOURCE_SUBDIR librmcs-unused
)
FetchContent_MakeAvailable(librmcs_soem)

set(soem_dir "${librmcs_soem_SOURCE_DIR}")
file(GLOB soem_sources "${soem_dir}/soem/*.c")
add_library(librmcs_soem STATIC ${soem_sources}
    "${soem_dir}/osal/linux/osal.c"
    "${soem_dir}/oshw/linux/nicdrv.c"
    "${soem_dir}/oshw/linux/oshw.c"
)
target_include_directories(librmcs_soem SYSTEM PUBLIC
    "${soem_dir}/soem" "${soem_dir}/osal" "${soem_dir}/osal/linux" "${soem_dir}/oshw/linux"
)
target_compile_definitions(librmcs_soem PUBLIC EC_VER2 PRIVATE _DEFAULT_SOURCE)
set_target_properties(librmcs_soem PROPERTIES C_VISIBILITY_PRESET hidden)
find_package(Threads REQUIRED)
target_link_libraries(librmcs_soem PUBLIC Threads::Threads rt)
install(TARGETS librmcs_soem ARCHIVE DESTINATION lib)
