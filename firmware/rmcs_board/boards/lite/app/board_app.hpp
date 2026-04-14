#pragma once

#include <cstddef>
#include <cstdint>

#include <hpm_common.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_mcan_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_spi_regs.h>
#include <hpm_uart_regs.h>

#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"

namespace librmcs::firmware::board {

#define BOARD_CAN0(prefix, suffix) prefix##1##suffix
#define BOARD_CAN1(prefix, suffix) prefix##0##suffix
#define BOARD_CAN2(prefix, suffix) prefix##3##suffix
#define BOARD_CAN3(prefix, suffix) prefix##2##suffix

uint32_t init_can(MCAN_Type* ptr);
void can_irq_handler(size_t board_can_index);

#define BOARD_UART_DBUS(prefix, suffix) prefix##7##suffix
#define BOARD_UART0(prefix, suffix)     prefix##0##suffix
#define BOARD_UART1(prefix, suffix)     prefix##2##suffix

uint32_t init_uart(UART_Type* ptr);
void uart_irq_handler(size_t board_uart_index);
void uart_dbus_irq_handler();

#define BOARD_SPI_BMI088(prefix, suffix) prefix##1##suffix

constexpr GpioPin kBmi088GyroIntPin = make_gpio_pin<gpiom_soc_gpio0, 'B', 12, false>();
constexpr GpioPin kBmi088AccelIntPin = make_gpio_pin<gpiom_soc_gpio0, 'B', 10, false>();
constexpr GpioPin kBmi088GyroChipSelectPin = make_gpio_pin<gpiom_core0_fast, 'B', 11, false>();
constexpr GpioPin kBmi088AccelChipSelectPin = make_gpio_pin<gpiom_core0_fast, 'B', 13, false>();

uint32_t init_spi(SPI_Type* ptr);
void spi_bmi088_irq_handler();

void bmi088_gyro_dataready_irq_handler();
void bmi088_accel_dataready_irq_handler();

constexpr GpioPin kUserHsFsSwitchPin = make_gpio_pin<gpiom_soc_gpio0, 'A', 31, false>();

void init_user_button_and_switch_pins();

} // namespace librmcs::firmware::board
