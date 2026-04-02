#include <board.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_soc.h>
#include <tusb.h>

#include "firmware/rmcs_board/bootloader/src/flash/validation.hpp"
#include "firmware/rmcs_board/bootloader/src/usb/dfu.hpp"
#include "firmware/rmcs_board/bootloader/src/usb/usb_descriptors.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/jump.hpp"

int main() {
    using namespace librmcs::firmware; // NOLINT(google-build-using-namespace)

    // Reset-time clocks already run CPU0 at 360 MHz, so board_init() would only bump it to
    // 480 MHz while adding avoidable startup latency on the direct-to-app path.
    const bool force_dfu = boot::BootMailbox::consume_enter_dfu_request();
    if (!force_dfu && flash::validate_app_image())
        utility::jump_to_app();

    board_init();
    board_init_usb(HPM_USB0);
    (void)usb::get_usb_descriptors();

    const tusb_rhport_init_t init_config{
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    utility::assert_always(tusb_rhport_init(0, &init_config));

    while (true) {
        tud_task();
        usb::Dfu::instance().poll();
    }
}
