sdk_inc(.)
sdk_src(board.c)
set_property(GLOBAL PROPERTY LIBRMCS_USB_PID 0xAF02)
sdk_compile_definitions(LIBRMCS_USB_PID=0xAF02)

if(LIBRMCS_BUILDING_FIRMWARE_APP)
    sdk_app_inc(app)
    sdk_app_src(app/peripherals.cpp app/status_led.cpp)
    sdk_linker_global_symbols("_flash_size=0x100000")
    set_property(GLOBAL PROPERTY LIBRMCS_APP_EXCLUDED_SOURCES
        can/can.cpp
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
