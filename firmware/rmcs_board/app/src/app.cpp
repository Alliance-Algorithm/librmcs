#include "firmware/rmcs_board/app/src/app.hpp"

#include <board.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_dma_mgr.h>

#include "board_app.hpp"
#if defined(LIBRMCS_ENABLE_ETHERCAT)
# include "firmware/rmcs_board/app/src/ecat/slave.hpp"
#endif
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

#if defined(LIBRMCS_ENABLE_ETHERCAT)
    board::init_user_button_and_switch_pins();
    const bool ethercat_selected = board::kModeSwitchPin.is_active();
    if (ethercat_selected)
        ecat::slave.init().start();
    else
        usb::vendor.init();
    usb::Device::init(TUSB_SPEED_HIGH, !ethercat_selected);
#else
    usb::vendor.init();
    board::init_user_button_and_switch_pins();
    const auto usb_speed =
        board::kUserHsFsSwitchPin.is_active() ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL;
    usb::Device::init(usb_speed);
#endif
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
        if (auto* vendor = usb::vendor.try_get())
            vendor->try_transmit();
#if defined(LIBRMCS_ENABLE_ETHERCAT)
        else
            ecat::slave->poll();
#endif

        Peripherals::poll();
        watchdog::watchdog->feed();
    }
}

core::protocol::Serializer& Peripherals::uplink_serializer() {
#if defined(LIBRMCS_ENABLE_ETHERCAT)
    if (auto* slave = ecat::slave.try_get())
        return slave->serializer();
#endif
    return usb::vendor->serializer();
}

} // namespace librmcs::firmware
