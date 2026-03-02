add_executable(c_board_bootloader)

set(C_BOARD_BOOTLOADER_TINYUSB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../bsp/tinyusb")
cmake_path(ABSOLUTE_PATH C_BOARD_BOOTLOADER_TINYUSB_ROOT NORMALIZE)
set(C_BOARD_BOOTLOADER_TINYUSB_COMMON_SOURCES
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/tusb.c"
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/common/tusb_fifo.c"
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/device/usbd.c"
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/device/usbd_control.c"
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c"
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c"
)

add_library(tinyusb_bootloader OBJECT)
target_sources(tinyusb_bootloader PRIVATE
    ${C_BOARD_BOOTLOADER_TINYUSB_COMMON_SOURCES}
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src/class/dfu/dfu_device.c"
)
target_include_directories(tinyusb_bootloader SYSTEM PUBLIC
    "${C_BOARD_BOOTLOADER_TINYUSB_ROOT}/src"
)
target_include_directories(tinyusb_bootloader PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_definitions(tinyusb_bootloader PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32F4
)
target_link_libraries(tinyusb_bootloader PUBLIC
    stm32cubemx
)

file(GLOB_RECURSE C_BOARD_BOOTLOADER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
target_sources(c_board_bootloader PRIVATE
    ${C_BOARD_BOOTLOADER_SOURCES}
)

target_include_directories(c_board_bootloader PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

c_board_apply_target_options(c_board_bootloader "STM32F407XX_BOOTLOADER.ld")
target_link_libraries(c_board_bootloader PRIVATE
    tinyusb_bootloader
)
