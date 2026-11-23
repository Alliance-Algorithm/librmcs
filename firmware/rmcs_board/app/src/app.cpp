#include "firmware/rmcs_board/app/src/app.hpp"

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>

#include "firmware/rmcs_board/app/src/led/led.hpp"
#include "firmware/rmcs_board/app/src/peripherals.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"
#include "firmware/rmcs_board/app/src/usb/dcd_recovery.hpp"
#include "firmware/rmcs_board/app/src/usb/device.hpp"
#include "firmware/rmcs_board/app/src/usb/vendor.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/rmcs_board/app/src/watchdog/watchdog.hpp"
#include "firmware/rmcs_board/common/boot_mailbox.hpp"

int main() { librmcs::firmware::app.init().run(); }

namespace librmcs::firmware {

App::App() {
    const utility::InterruptLockGuard guard;

    board_init();
    board_init_usb();
    dma_mgr_init();
    boot::BootMailbox::clear();

    watchdog::watchdog.init();

    led::led.init();
    timer::timer.init();

    usb::vendor.init();
    usb::Device::init();
    Peripherals::init();
}

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        if (usb_device_recovery_required()) {
            while (true) {}
        }

        tud_task();
        usb::vendor->try_transmit();

        Peripherals::poll();
        watchdog::watchdog->feed();
    }
}

core::protocol::Serializer& Peripherals::uplink_serializer() { return usb::vendor->serializer(); }

} // namespace librmcs::firmware
