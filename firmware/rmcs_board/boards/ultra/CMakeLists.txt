sdk_inc(.)
sdk_src(board.c)
set_property(GLOBAL PROPERTY LIBRMCS_USB_PID 0xAF02)
sdk_compile_definitions(LIBRMCS_USB_PID=0xAF02)

# HPM5E31 loses SYSCTL GPRs on software reset; use the retained PMIC registers.
sdk_compile_definitions(BOARD_BOOT_MAILBOX_PGPR=HPM_PGPR0)

if(LIBRMCS_BUILDING_FIRMWARE_APP)
    sdk_app_inc(app)
    sdk_app_src(app/board_app.cpp app/peripherals.cpp app/status_led.cpp app/status_buzzer.cpp)
    if(LIBRMCS_ENABLE_ETHERCAT)
        if(NOT LIBRMCS_ULTRA_ECAT_WIRING_CONFIRMED)
            message(FATAL_ERROR "Verify Ultra EtherCAT REFCLK/RX_ER routing before enabling it.")
        endif()
        sdk_app_src(app/ethercat_phy.cpp)
        sdk_compile_definitions(HPM_ESC_DRV_DEFAULT_RETRY_COUNT=100000U)
    endif()
    sdk_linker_global_symbols("_flash_size=0x100000")
    set_property(GLOBAL PROPERTY LIBRMCS_APP_EXCLUDED_SOURCES
        buzzer/status_buzzer.cpp
        gpio/gpio.cpp
        peripherals.cpp
        spi/bmi088/accel.cpp
        spi/bmi088/gyro.cpp
        spi/bmi088/temperature.cpp
        spi/spi.cpp
        led/status_led.cpp
        uart/uart.cpp
    )
endif()
