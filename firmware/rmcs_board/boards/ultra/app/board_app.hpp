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

#define BOARD_UART_DBUS(prefix, suffix) prefix##2##suffix
#define BOARD_UART0(prefix, suffix)     prefix##6##suffix
#define BOARD_UART1(prefix, suffix)     prefix##0##suffix
#define BOARD_UART2(prefix, suffix)     prefix##7##suffix

#define BOARD_SPI_BMI088(prefix, suffix) prefix##3##suffix

inline constexpr GpioPin kBmi088GyroIntPin = make_gpio_pin<gpiom_soc_gpio0, 'D', 7, false>();
inline constexpr GpioPin kBmi088AccelIntPin = make_gpio_pin<gpiom_soc_gpio0, 'D', 6, false>();
inline constexpr GpioPin kBmi088GyroChipSelectPin =
    make_gpio_pin<gpiom_core0_fast, 'C', 19, false>();
inline constexpr GpioPin kBmi088AccelChipSelectPin =
    make_gpio_pin<gpiom_core0_fast, 'C', 21, false>();

void spi_bmi088_irq_handler();
void bmi088_gyro_dataready_irq_handler();
void bmi088_accel_dataready_irq_handler();

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
