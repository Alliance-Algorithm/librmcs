#include "firmware/src/app.hpp"

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>

#include "firmware/src/can/can.hpp"
#include "firmware/src/gpio/gpio.hpp"
#include "firmware/src/spi/bmi088/accel.hpp"
#include "firmware/src/spi/bmi088/gyro.hpp"
#include "firmware/src/uart/uart.hpp"
#include "firmware/src/usb/vendor.hpp"
#include "firmware/src/utility/interrupt_lock_guard.hpp"

int main() { librmcs::firmware::app.init().run(); }

namespace librmcs::firmware {

App::App() {
    utility::InterruptLockGuard guard;

    board_init();
    board_init_usb(HPM_USB0);
    dma_mgr_init();

    can::can0.init();
    can::can1.init();
    can::can2.init();
    can::can3.init();
    uart::uart3.init();

    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    gpio::init_bmi088_interrupts();

    usb::vendor.init();
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();
        usb::vendor->try_transmit();
        uart::uart3->try_transmit();
    }
}

} // namespace librmcs::firmware
