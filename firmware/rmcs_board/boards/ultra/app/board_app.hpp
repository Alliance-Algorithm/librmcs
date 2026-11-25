#pragma once

#include <hpm_common.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_ioc_regs.h>

#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"

namespace librmcs::firmware::board {

inline constexpr auto kModeSwitchPin = make_gpio_pin<gpiom_soc_gpio0, 'F', 17, false>();
#if !defined(LIBRMCS_ENABLE_ETHERCAT)
inline constexpr auto kUserHsFsSwitchPin = kModeSwitchPin;
#endif

inline void init_user_button_and_switch_pins() {
    kModeSwitchPin.configure_controller();
    kModeSwitchPin.configure_as_input();
    kModeSwitchPin.configure_ioc_function();
    kModeSwitchPin.configure_pad_control(IOC_PAD_PAD_CTL_HYS_SET(1));
}

} // namespace librmcs::firmware::board
