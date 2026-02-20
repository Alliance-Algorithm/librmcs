#include "firmware/c_board/src/app.hpp"

#include <device/usbd.h>
#include <main.h>

#include "firmware/c_board/src/can/can.hpp"
#include "firmware/c_board/src/led/led.hpp"
#include "firmware/c_board/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/src/spi/bmi088/gyro.hpp"
#include "firmware/c_board/src/uart/uart.hpp"
#include "firmware/c_board/src/usb/vendor.hpp"
#include "firmware/c_board/src/utility/interrupt_lock.hpp"

extern "C" {
void AppEntry() { librmcs::firmware::app.init().run(); }
}

namespace librmcs::firmware {

App::App() {
    led::led.init();
    usb::vendor.init();
    can::can1.init();
    can::can2.init();
    uart::uart1.init();
    uart::uart2.init();
    uart::uart_dbus.init();
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    utility::InterruptMutex::unlock();
}

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();

        usb::vendor->try_transmit();
        can::can1->try_transmit();
        usb::vendor->try_transmit();
        can::can2->try_transmit();
        usb::vendor->try_transmit();
        uart::uart1->try_transmit();
        usb::vendor->try_transmit();
        uart::uart2->try_transmit();
        usb::vendor->try_transmit();
        uart::uart_dbus->try_transmit();
    }
}

} // namespace librmcs::firmware
