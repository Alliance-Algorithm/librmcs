#include "board_app.hpp"

#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpio_regs.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_mcan_regs.h>
#include <hpm_pmic_iomux.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_spi_regs.h>
#include <hpm_uart_regs.h>

namespace librmcs::firmware::board {
namespace {

uint32_t init_can_clock(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        clock_add_to_group(clock_can0, 0);
        clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can0);
    }
    if (ptr == HPM_MCAN1) {
        clock_add_to_group(clock_can1, 0);
        clock_set_source_divider(clock_can1, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can1);
    }
    if (ptr == HPM_MCAN2) {
        clock_add_to_group(clock_can2, 0);
        clock_set_source_divider(clock_can2, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can2);
    }
    if (ptr == HPM_MCAN3) {
        clock_add_to_group(clock_can3, 0);
        clock_set_source_divider(clock_can3, clk_src_pll1_clk0, 10);
        return clock_get_frequency(clock_can3);
    }
    return 0;
}

inline uint32_t init_uart_clock(UART_Type* ptr) {
    if (ptr == HPM_UART0) {
        clock_add_to_group(clock_uart0, 0);
        return clock_get_frequency(clock_uart0);
    }
    if (ptr == HPM_UART1) {
        clock_add_to_group(clock_uart1, 0);
        return clock_get_frequency(clock_uart1);
    }
    if (ptr == HPM_UART2) {
        clock_add_to_group(clock_uart2, 0);
        return clock_get_frequency(clock_uart2);
    }
    if (ptr == HPM_UART3) {
        clock_add_to_group(clock_uart3, 0);
        return clock_get_frequency(clock_uart3);
    }
    if (ptr == HPM_UART5) {
        clock_add_to_group(clock_uart5, 0);
        return clock_get_frequency(clock_uart5);
    }
    return 0;
}

inline uint32_t init_spi_clock(SPI_Type* ptr) {
    if (ptr == HPM_SPI1) {
        clock_add_to_group(clock_spi1, 0);
        return clock_get_frequency(clock_spi1);
    }
    if (ptr == HPM_SPI2) {
        clock_add_to_group(clock_spi2, 0);
        return clock_get_frequency(clock_spi2);
    }
    if (ptr == HPM_SPI3) {
        clock_add_to_group(clock_spi3, 0);
        return clock_get_frequency(clock_spi3);
    }
    return 0;
}

} // namespace

uint32_t init_can(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        HPM_IOC->PAD[IOC_PAD_PA17].FUNC_CTL = IOC_PA17_FUNC_CTL_MCAN0_RXD;
        HPM_IOC->PAD[IOC_PAD_PA16].FUNC_CTL = IOC_PA16_FUNC_CTL_MCAN0_TXD;
    } else if (ptr == HPM_MCAN1) {
        HPM_IOC->PAD[IOC_PAD_PA20].FUNC_CTL = IOC_PA20_FUNC_CTL_MCAN1_RXD;
        HPM_IOC->PAD[IOC_PAD_PA21].FUNC_CTL = IOC_PA21_FUNC_CTL_MCAN1_TXD;
    } else if (ptr == HPM_MCAN2) {
        HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_MCAN2_RXD;
        HPM_IOC->PAD[IOC_PAD_PA24].FUNC_CTL = IOC_PA24_FUNC_CTL_MCAN2_TXD;
    } else if (ptr == HPM_MCAN3) {
        HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_MCAN3_RXD;
        HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_MCAN3_TXD;
    }
    return init_can_clock(ptr);
}

uint32_t init_uart(UART_Type* ptr) {
    if (ptr == HPM_UART0) {
        HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_UART0_TXD;
        HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_UART0_RXD;
    } else if (ptr == HPM_UART1) {
        HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART1_TXD;
        HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_GPIO_Y_07;
        HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART1_RXD;
        HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_GPIO_Y_06;
        HPM_IOC->PAD[IOC_PAD_PY05].FUNC_CTL = IOC_PY05_FUNC_CTL_UART1_DE;
        HPM_PIOC->PAD[IOC_PAD_PY05].FUNC_CTL = PIOC_PY05_FUNC_CTL_SOC_GPIO_Y_05;
    } else if (ptr == HPM_UART2) {
        HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_UART2_RXD;
    } else if (ptr == HPM_UART3) {
        HPM_IOC->PAD[IOC_PAD_PA15].FUNC_CTL = IOC_PA15_FUNC_CTL_UART3_TXD;
        HPM_IOC->PAD[IOC_PAD_PA14].FUNC_CTL = IOC_PA14_FUNC_CTL_UART3_RXD;
    } else if (ptr == HPM_UART5) {
        HPM_IOC->PAD[IOC_PAD_PA23].FUNC_CTL = IOC_PA23_FUNC_CTL_UART5_TXD;
        HPM_IOC->PAD[IOC_PAD_PA22].FUNC_CTL = IOC_PA22_FUNC_CTL_UART5_RXD;
    }
    return init_uart_clock(ptr);
}

uint32_t init_spi(SPI_Type* ptr) {
    if (ptr == HPM_SPI1) { // SPI1: W25Q128
        HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL =
            IOC_PA27_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;

        HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_SPI1_MOSI;
        HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_SPI1_MISO;
        HPM_IOC->PAD[IOC_PAD_PA26].FUNC_CTL = IOC_PA26_FUNC_CTL_SPI1_CS_0;

    } else if (ptr == HPM_SPI2) { // SPI2: BMI088
        HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL =
            IOC_PB11_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;

        HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_SPI2_MOSI;
        HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_SPI2_MISO;

        kBmi088GyroIntPin.configure_controller();
        kBmi088GyroIntPin.configure_ioc_function();
        kBmi088GyroIntPin.configure_as_input();
        kBmi088GyroIntPin.configure_interrupt(gpio_interrupt_trigger_edge_falling);
        kBmi088GyroIntPin.clear_interrupt_flag();
        kBmi088GyroIntPin.enable_interrupt();
        intc_m_enable_irq_with_priority(IRQn_GPIO0_B, 1);

        kBmi088AccelIntPin.configure_controller();
        kBmi088AccelIntPin.configure_pioc_function();
        kBmi088AccelIntPin.configure_ioc_function();
        kBmi088AccelIntPin.configure_as_input();
        kBmi088AccelIntPin.configure_interrupt(gpio_interrupt_trigger_edge_falling);
        kBmi088AccelIntPin.clear_interrupt_flag();
        kBmi088AccelIntPin.enable_interrupt();
        intc_m_enable_irq_with_priority(IRQn_GPIO0_Y, 1);

        kBmi088GyroChipSelectPin.configure_controller();
        kBmi088GyroChipSelectPin.configure_ioc_function();
        kBmi088GyroChipSelectPin.set_active(false);
        kBmi088GyroChipSelectPin.configure_as_output();

        kBmi088AccelChipSelectPin.configure_controller();
        kBmi088AccelChipSelectPin.configure_ioc_function();
        kBmi088AccelChipSelectPin.set_active(false);
        kBmi088AccelChipSelectPin.configure_as_output();

    } else if (ptr == HPM_SPI3) { // SPI3: PinSocket Output
        HPM_IOC->PAD[IOC_PAD_PA11].FUNC_CTL =
            IOC_PA11_FUNC_CTL_SPI3_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
        HPM_IOC->PAD[IOC_PAD_PA12].FUNC_CTL = IOC_PA12_FUNC_CTL_SPI3_MISO;
        HPM_IOC->PAD[IOC_PAD_PA13].FUNC_CTL = IOC_PA13_FUNC_CTL_SPI3_MOSI;

        HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_SPI3_CS_0;
        HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_SPI3_CS_1;
    }

    return init_spi_clock(ptr);
}

void init_user_button_and_switch_pins() {
    kUserButtonPin.configure_controller();
    kUserButtonPin.configure_pioc_function();
    kUserButtonPin.configure_ioc_function();
    kUserButtonPin.configure_pad_control(
        IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
        IOC_PAD_PAD_CTL_PS_SET(1) | // Pull select - Pull up
        IOC_PAD_PAD_CTL_HYS_SET(1)  // Enable Schmitt trigger
    );
    kUserButtonPin.configure_as_input();

    kUserHsFsSwitchPin.configure_controller();
    kUserHsFsSwitchPin.configure_pioc_function();
    kUserHsFsSwitchPin.configure_ioc_function();
    kUserHsFsSwitchPin.configure_pad_control(
        IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
        IOC_PAD_PAD_CTL_PS_SET(0) | // Pull select - Pull down
        IOC_PAD_PAD_CTL_HYS_SET(1)  // enable Schmitt trigger
    );
    kUserHsFsSwitchPin.configure_as_input();
}

void init_ws2812_pin() {
    // WS2812 is not populated until next hardware revision.
    // Configure as input to prevent driving the line.
    kWs2812Pin.configure_controller();
    kWs2812Pin.configure_as_gpio();
    kWs2812Pin.configure_as_input();
}

void init_gpio_pins() {
    // Intentionally empty: No PIOC pins need to be configured.
}

uint32_t init_tick_clock() {
    clock_add_to_group(clock_gptmr1, 0);
    return clock_get_frequency(clock_gptmr1);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_A, gpio_a_isr)
void gpio_a_isr() { gpio_irq_handler(GPIO_DI_GPIOA); }

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B, gpio_bmi088_int_gyro_isr)
void gpio_bmi088_int_gyro_isr() {
    if (kBmi088GyroIntPin.check_clear_interrupt_flag()) {
        bmi088_gyro_dataready_irq_handler();
    }
    gpio_irq_handler(GPIO_DI_GPIOB);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Y, gpio_bmi088_int_accel_isr)
void gpio_bmi088_int_accel_isr() {
    if (kBmi088AccelIntPin.check_clear_interrupt_flag()) {
        bmi088_accel_dataready_irq_handler();
    }
    gpio_irq_handler(GPIO_DI_GPIOY);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR1, tick_isr)
void tick_isr() { tick_clock_irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN0(IRQn_MCAN, ), can0_isr)
void can0_isr() { can_irq_handler(0); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN1(IRQn_MCAN, ), can1_isr)
void can1_isr() { can_irq_handler(1); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN2(IRQn_MCAN, ), can2_isr)
void can2_isr() { can_irq_handler(2); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN3(IRQn_MCAN, ), can3_isr)
void can3_isr() { can_irq_handler(3); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART0(IRQn_UART, ), uart0_isr)
void uart0_isr() { uart_irq_handler(0); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART1(IRQn_UART, ), uart1_isr)
void uart1_isr() { uart_irq_handler(1); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART2(IRQn_UART, ), uart2_isr)
void uart2_isr() { uart_irq_handler(2); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART3(IRQn_UART, ), uart3_isr)
void uart3_isr() { uart_irq_handler(3); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART_DBUS(IRQn_UART, ), uart_dbus_isr)
void uart_dbus_isr() { uart_dbus_irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_SPI_BMI088(IRQn_SPI, ), spi_isr)
void spi_isr() { spi_bmi088_irq_handler(); }

} // namespace librmcs::firmware::board
