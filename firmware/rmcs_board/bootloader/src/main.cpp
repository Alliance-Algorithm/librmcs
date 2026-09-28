#include <board.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <tusb.h>

#include "firmware/rmcs_board/app/src/watchdog/watchdog.hpp"
#include "firmware/rmcs_board/bootloader/src/flash/validation.hpp"
#include "firmware/rmcs_board/bootloader/src/usb/dfu.hpp"
#include "firmware/rmcs_board/bootloader/src/usb/usb_descriptors.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/jump.hpp"

// From bsp/hpm_sdk/components/usb/device/hpm_usb_device.c: set once the USB bus reset
// handshake timed out and the USB interrupts were masked. Declared here instead of
// including hpm_usb_device.h, whose C bit-field declarations do not compile as C++.
extern "C" bool usb_device_recovery_required();

int main() {
    using namespace librmcs::firmware; // NOLINT(google-build-using-namespace)

    const bool force_stay = board_check_bootloader_force_stay_requested();

#if LIBRMCS_BOOTLOADER_MODE_AUTO
    if (!force_stay && !boot::BootMailbox::consume_enter_dfu_request()
        && flash::validate_app_image())
#else
    if (!force_stay && boot::BootMailbox::consume_boot_app_once_request()
        && flash::validate_app_image())
#endif
        utility::jump_to_app();

    // Reset-time clocks already run CPU0 at 360 MHz, so board_init() would only bump it to
    // 480 MHz while adding avoidable startup latency on the direct-to-app path.
    board_init();

    // Start the watchdog as early as possible after the jump decision so that unbounded waits
    // in board_init_usb() / tusb_rhport_init() are also covered.
    watchdog::watchdog.init();

    board_init_usb();
    (void)usb::get_usb_descriptors();

    const tusb_rhport_init_t init_config{
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    utility::assert_always(tusb_rhport_init(0, &init_config));

    while (true) {
        if (usb_device_recovery_required()) {
            // USB controller wedged during bus reset (interrupts already masked).
            // Stop feeding so the watchdog resets the SoC instead of hanging in DFU.
            while (true) {}
        }

        tud_task();
        usb::Dfu::instance().poll();
        watchdog::watchdog->feed();
    }
}
