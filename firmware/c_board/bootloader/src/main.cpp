#include <cstdint>

#include <device/usbd.h>
#include <gpio.h>
#include <main.h>
#include <system_stm32f4xx.h>
#include <tusb.h>
#include <usb_otg.h>

#include "firmware/c_board/app/src/watchdog/watchdog.hpp"
#include "firmware/c_board/bootloader/src/flash/layout.hpp"
#include "firmware/c_board/bootloader/src/flash/validation.hpp"
#include "firmware/c_board/bootloader/src/usb/dfu.hpp"
#include "firmware/c_board/bootloader/src/utility/assert.hpp"
#include "firmware/c_board/bootloader/src/utility/boot_mailbox.hpp"
#include "firmware/c_board/bootloader/src/utility/jump.hpp"

// From bsp/tinyusb/src/portable/synopsys/dwc2/dwc2_common.c: set once a bounded DWC2
// hardware wait timed out and the controller interrupts were masked.
extern "C" bool dwc2_recovery_required();

namespace {

void bootloader_delay_us(uint32_t delay_us) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t start = DWT->CYCCNT;
    const uint32_t delay_cycles = (SystemCoreClock / 1000000U) * delay_us;
    while ((DWT->CYCCNT - start) < delay_cycles) {}
}

bool bootloader_check_bootloader_force_stay_requested() {
    for (uint32_t sample_index = 0; sample_index < 4; ++sample_index) {
        bootloader_delay_us(250);
        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_RESET)
            return false;
    }

    return true;
}

} // namespace

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    const bool force_stay = bootloader_check_bootloader_force_stay_requested();

    MX_USB_OTG_FS_PCD_Init();

    using namespace librmcs::firmware; // NOLINT(google-build-using-namespace)

    const bool force_dfu = utility::boot_mailbox.consume_enter_dfu_request();
    if (!force_stay && !force_dfu) {
        if (flash::validate_app_image())
            utility::jump_to_app(flash::kAppStartAddress);
    }

    // Start the watchdog only when staying in the bootloader: the app reconfigures the same
    // (unstoppable) IWDG on its own, and starting it earlier would only narrow the app's window.
    watchdog::watchdog.init();

    utility::assert_always(tusb_rhport_init(0, nullptr));

    while (true) {
        if (dwc2_recovery_required()) {
            // USB controller wedged during a hardware wait (interrupts already masked).
            // Stop feeding so the watchdog resets the SoC instead of hanging in DFU.
            while (true) {}
        }

        tud_task();
        usb::Dfu::instance().poll();
        watchdog::watchdog->feed();
    }
}
