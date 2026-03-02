#include <device/usbd.h>
#include <gpio.h>
#include <main.h>
#include <tusb.h>
#include <usb_otg.h>

#include "firmware/c_board/bootloader/src/flash/layout.hpp"
#include "firmware/c_board/bootloader/src/flash/validation.hpp"
#include "firmware/c_board/bootloader/src/usb/dfu.hpp"
#include "firmware/c_board/bootloader/src/utility/assert.hpp"
#include "firmware/c_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/c_board/bootloader/src/utility/jump.hpp"

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USB_OTG_FS_PCD_Init();

    using namespace librmcs::firmware; // NOLINT(google-build-using-namespace)

    const bool force_dfu = utility::boot_mailbox.consume_enter_dfu_request();
    if (!force_dfu) {
        if (flash::validate_app_image()) {
            utility::jump_to_app(flash::kAppStartAddress);
            utility::assert_failed_always();
        }
    }

    utility::assert_always(tusb_rhport_init(0, nullptr));

    while (true) {
        tud_task();
        usb::Dfu::instance().poll();
    }
}
