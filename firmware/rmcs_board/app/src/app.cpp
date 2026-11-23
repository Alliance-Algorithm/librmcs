#include "firmware/rmcs_board/app/src/app.hpp"

#include <board.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>

#include "firmware/rmcs_board/app/src/led/led.hpp"
#include "firmware/rmcs_board/app/src/peripherals.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"
#include "firmware/rmcs_board/app/src/usb/dcd_recovery.hpp"
#include "firmware/rmcs_board/app/src/usb/device.hpp"
#if defined(LIBRMCS_ENABLE_ETHERCAT)
# include "firmware/rmcs_board/app/src/ecat/slave.hpp"
#else
# include "firmware/rmcs_board/app/src/usb/vendor.hpp"
#endif
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

#if defined(LIBRMCS_ENABLE_ETHERCAT)
    ecat::slave.init().start();
#else
    usb::vendor.init();
#endif
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
#if defined(LIBRMCS_ENABLE_ETHERCAT)
        ecat::slave->poll();
#else
        usb::vendor->try_transmit();
#endif

        Peripherals::poll();
        watchdog::watchdog->feed();
    }
}

core::protocol::Serializer& Peripherals::uplink_serializer() {
#if defined(LIBRMCS_ENABLE_ETHERCAT)
    return ecat::slave->serializer();
#else
    return usb::vendor->serializer();
#endif
}

} // namespace librmcs::firmware
