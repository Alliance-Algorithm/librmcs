#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <hpm_common.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_iomux.h>
#include <hpm_mcan_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_spi_regs.h>
#include <hpm_uart_regs.h>

#include "core/include/librmcs/spec/rmcs_board_pro/gpio.hpp" // IWYU pragma: export
#include "firmware/rmcs_board/app/src/gpio/analog_gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"

namespace librmcs::firmware::board {

namespace spec = librmcs::spec::rmcs_board_pro;

#define BOARD_CAN0(prefix, suffix) prefix##3##suffix
#define BOARD_CAN1(prefix, suffix) prefix##2##suffix
#define BOARD_CAN2(prefix, suffix) prefix##1##suffix
#define BOARD_CAN3(prefix, suffix) prefix##0##suffix

uint32_t init_can(MCAN_Type* ptr);
void can_irq_handler(size_t board_can_index);

#define BOARD_UART_DBUS(prefix, suffix) prefix##2##suffix
#define BOARD_UART0(prefix, suffix)     prefix##5##suffix
#define BOARD_UART1(prefix, suffix)     prefix##3##suffix
#define BOARD_UART2(prefix, suffix)     prefix##0##suffix
#define BOARD_UART3(prefix, suffix)     prefix##1##suffix

uint32_t init_uart(UART_Type* ptr);
void uart_irq_handler(size_t board_uart_index);
void uart_dbus_irq_handler();

#define BOARD_SPI_BMI088(prefix, suffix) prefix##2##suffix

constexpr GpioPin kBmi088GyroIntPin = make_gpio_pin<gpiom_soc_gpio0, 'B', 15, false>();
constexpr GpioPin kBmi088AccelIntPin = make_gpio_pin<gpiom_soc_gpio0, 'Y', 0, false>();
constexpr GpioPin kBmi088GyroChipSelectPin = make_gpio_pin<gpiom_core0_fast, 'B', 10, false>();
constexpr GpioPin kBmi088AccelChipSelectPin = make_gpio_pin<gpiom_core0_fast, 'B', 14, false>();
constexpr GpioPin kBmi088HeaterPin = make_gpio_pin<gpiom_soc_gpio0, 'Y', 1, true>();

uint32_t init_spi(SPI_Type* ptr);
void spi_bmi088_irq_handler();

void bmi088_gyro_dataready_irq_handler();
void bmi088_accel_dataready_irq_handler();

constexpr GpioPin kUserButtonPin = make_gpio_pin<gpiom_soc_gpio0, 'Y', 3, false>();
constexpr GpioPin kUserHsFsSwitchPin = make_gpio_pin<gpiom_soc_gpio0, 'Y', 4, false>();

void init_user_button_and_switch_pins();

inline constexpr AnalogGpioPin kGpioHardwareDescriptors[]{
    {make_gpio_pin<gpiom_soc_gpio0, 'B', 0>(), HPM_PWM0_BASE, IOC_PB00_FUNC_CTL_PWM0_P_0, 0},
    {make_gpio_pin<gpiom_soc_gpio0, 'B', 1>(), HPM_PWM0_BASE, IOC_PB01_FUNC_CTL_PWM0_P_1, 1},
    {make_gpio_pin<gpiom_soc_gpio0, 'B', 2>(), HPM_PWM0_BASE, IOC_PB02_FUNC_CTL_PWM0_P_2, 2},
    {make_gpio_pin<gpiom_soc_gpio0, 'B', 3>(), HPM_PWM0_BASE, IOC_PB03_FUNC_CTL_PWM0_P_3, 3},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 9>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 10>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 11>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 12>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 13>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 18>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 19>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 22>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 23>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 14>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 15>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 1>()},
    {make_gpio_pin<gpiom_soc_gpio0, 'A', 0>()},
};
static_assert(board::spec::kGpioDescriptors.size() == std::size(board::kGpioHardwareDescriptors));

void init_gpio_pins();
void gpio_irq_handler(uint32_t port_index);

} // namespace librmcs::firmware::board
