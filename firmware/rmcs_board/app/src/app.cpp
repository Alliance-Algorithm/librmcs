#include "firmware/rmcs_board/app/src/app.hpp"

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>
#include <hpm_soc.h>

#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio.hpp"
#include "firmware/rmcs_board/app/src/i2c/i2c.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"
#include "firmware/rmcs_board/app/src/usb/vendor.hpp"
#include "firmware/rmcs_board/app/src/utility/boot_mailbox.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"

int main() { librmcs::firmware::app.init().run(); }

namespace librmcs::firmware {

App::App() {
    const utility::InterruptLockGuard guard;

    board_init();
    board_init_usb(HPM_USB0);
    dma_mgr_init();
    boot::BootMailbox::clear();

    can::can0.init();
    can::can1.init();
    can::can2.init();
    can::can3.init();
    uart::uart0.init();
    uart::uart1.init();
    uart::uart2.init();
    uart::uart3.init();
    uart::uart_dbus.init();
    i2c::i2c0.init();

    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    gpio::init_bmi088_interrupts();

    usb::vendor.init();
}

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();
        usb::vendor->try_transmit();
        uart::uart0->try_transmit();
        uart::uart1->try_transmit();
        uart::uart2->try_transmit();
        uart::uart3->try_transmit();
        uart::uart_dbus->try_transmit();
    }
}

} // namespace librmcs::firmware
