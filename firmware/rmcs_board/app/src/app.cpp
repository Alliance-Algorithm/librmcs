#include "firmware/rmcs_board/app/src/app.hpp"

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>

#include "firmware/rmcs_board/app/src/can/can.hpp"
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
    board_init_usb();
    dma_mgr_init();
    boot::BootMailbox::clear();

    usb::vendor.init();

    for (auto& can : can::can_array)
        can.init();

    uart::uart_dbus.init();
    i2c::i2c0.init();
    for (auto& board_uart : uart::uart_array)
        board_uart.init();

    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
}

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();
        usb::vendor->try_transmit();

        for (auto& board_uart : uart::uart_array)
            board_uart->try_transmit();
    }
}

} // namespace librmcs::firmware
