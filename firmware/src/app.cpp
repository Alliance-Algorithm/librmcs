#include "firmware/src/app.hpp"
#include "firmware/src/can/can.hpp"
#include "firmware/src/usb/usb_descriptors.hpp"
#include "firmware/src/usb/vendor.hpp"
#include "firmware/src/utility/interrupt_lock_guard.hpp"

#include <board.h>
#include <device/usbd.h>
#include <tusb.h>

int main() { librmcs::firmware::app.init().run(); }

namespace librmcs::firmware {

App::App() {
    utility::InterruptLockGuard guard;

    board_init();
    board_init_usb(HPM_USB0);

    can::can0.init();
    can::can1.init();
    can::can2.init();
    can::can3.init();

    usb::usb_descriptors.init();
    tusb_init();
    usb::vendor.init();
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();
        usb::vendor->try_transmit();
    }
}

} // namespace librmcs::firmware
