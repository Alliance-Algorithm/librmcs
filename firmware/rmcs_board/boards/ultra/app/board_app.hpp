#pragma once

#include <hpm_common.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_ioc_regs.h>

#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"

namespace librmcs::firmware::board {

// Logical CAN ports follow the connector positions, not the MCAN instance numbers.
#define BOARD_CAN0(prefix, suffix) prefix##2##suffix
#define BOARD_CAN1(prefix, suffix) prefix##1##suffix
#define BOARD_CAN2(prefix, suffix) prefix##3##suffix
#define BOARD_CAN3(prefix, suffix) prefix##0##suffix

inline constexpr auto kUserHsFsSwitchPin = make_gpio_pin<gpiom_soc_gpio0, 'F', 17, false>();

inline void init_user_button_and_switch_pins() {
    kUserHsFsSwitchPin.configure_controller();
    kUserHsFsSwitchPin.configure_as_input();
    kUserHsFsSwitchPin.configure_ioc_function();
    kUserHsFsSwitchPin.configure_pad_control(IOC_PAD_PAD_CTL_HYS_SET(1));
}

} // namespace librmcs::firmware::board
