#include "firmware/rmcs_board/app/src/usb/device.hpp"

#include <tusb.h>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/rmcs_board/common/boot_mailbox.hpp"

namespace librmcs::firmware::usb {

void Device::init() {
    usb_descriptors.init();
    board::init_user_button_and_switch_pins();
    const tusb_rhport_init_t init_config{
        .role = TUSB_ROLE_DEVICE,
        .speed = board::kUserHsFsSwitchPin.is_active() ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL,
    };
    core::utility::assert_always(tusb_rhport_init(0, &init_config));
}

extern "C" void tud_dfu_runtime_reboot_to_dfu_cb() { boot::BootMailbox::reboot_to_bootloader(); }

} // namespace librmcs::firmware::usb
