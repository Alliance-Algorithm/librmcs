#include "firmware/rmcs_board/app/src/usb/device.hpp"

#include <common/tusb_types.h>
#include <tusb.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/rmcs_board/common/boot_mailbox.hpp"

namespace librmcs::firmware::usb {

void Device::init(tusb_speed_t speed, bool vendor_enabled) {
    usb_descriptors.init().set_vendor_enabled(vendor_enabled);
    const tusb_rhport_init_t init_config{
        .role = TUSB_ROLE_DEVICE,
        .speed = speed,
    };
    core::utility::assert_always(tusb_rhport_init(0, &init_config));
}

extern "C" void tud_dfu_runtime_reboot_to_dfu_cb() { boot::BootMailbox::reboot_to_bootloader(); }

} // namespace librmcs::firmware::usb
