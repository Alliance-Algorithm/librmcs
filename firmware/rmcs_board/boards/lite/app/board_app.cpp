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
    if (ptr == HPM_UART2) {
        clock_add_to_group(clock_uart2, 0);
        return clock_get_frequency(clock_uart2);
    }
    if (ptr == HPM_UART7) {
        clock_add_to_group(clock_uart7, 0);
        return clock_get_frequency(clock_uart7);
    }
    return 0;
}

inline uint32_t init_spi_clock(SPI_Type* ptr) {
    if (ptr == HPM_SPI1) {
        clock_add_to_group(clock_spi1, 0);
        return clock_get_frequency(clock_spi1);
    }
    return 0;
}

} // namespace

uint32_t init_can(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_MCAN0_RXD;
        HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_MCAN0_TXD;
    } else if (ptr == HPM_MCAN1) {
        HPM_IOC->PAD[IOC_PAD_PA04].FUNC_CTL = IOC_PA04_FUNC_CTL_MCAN1_RXD;
        HPM_IOC->PAD[IOC_PAD_PA05].FUNC_CTL = IOC_PA05_FUNC_CTL_MCAN1_TXD;
    } else if (ptr == HPM_MCAN2) {
        HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_MCAN2_RXD;
        HPM_IOC->PAD[IOC_PAD_PA08].FUNC_CTL = IOC_PA08_FUNC_CTL_MCAN2_TXD;
    } else if (ptr == HPM_MCAN3) {
        HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PB14_FUNC_CTL_MCAN3_RXD;
        HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_MCAN3_TXD;
    }
    return init_can_clock(ptr);
}

uint32_t init_uart(UART_Type* ptr) {
    if (ptr == HPM_UART0) {
        HPM_IOC->PAD[IOC_PAD_PY01].FUNC_CTL = IOC_PY01_FUNC_CTL_UART0_RXD;
        HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = PIOC_PY01_FUNC_CTL_SOC_GPIO_Y_01;
        HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_UART0_TXD;
        HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = PIOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
    } else if (ptr == HPM_UART2) {
        HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_UART2_RXD;
        HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PB08_FUNC_CTL_UART2_TXD;
    } else if (ptr == HPM_UART7) {
        HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_UART7_RXD;
    }
    return init_uart_clock(ptr);
}

uint32_t init_spi(SPI_Type* ptr) {
    if (ptr == HPM_SPI1) {
        HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL =
            IOC_PA27_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
        HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_SPI1_MOSI;
        HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_SPI1_MISO;

        kBmi088GyroIntPin.configure_controller();
        kBmi088GyroIntPin.configure_ioc_function();
        kBmi088GyroIntPin.configure_as_input();
        kBmi088GyroIntPin.configure_interrupt(gpio_interrupt_trigger_edge_falling);
        kBmi088GyroIntPin.clear_interrupt_flag();
        kBmi088GyroIntPin.enable_interrupt();

        kBmi088AccelIntPin.configure_controller();
        kBmi088AccelIntPin.configure_ioc_function();
        kBmi088AccelIntPin.configure_as_input();
        kBmi088AccelIntPin.configure_interrupt(gpio_interrupt_trigger_edge_falling);
        kBmi088AccelIntPin.clear_interrupt_flag();
        kBmi088AccelIntPin.enable_interrupt();

        intc_m_enable_irq_with_priority(IRQn_GPIO0_B, 1);

        kBmi088GyroChipSelectPin.configure_controller();
        kBmi088GyroChipSelectPin.configure_ioc_function();
        kBmi088GyroChipSelectPin.set_active(false);
        kBmi088GyroChipSelectPin.configure_as_output();

        kBmi088AccelChipSelectPin.configure_controller();
        kBmi088AccelChipSelectPin.configure_ioc_function();
        kBmi088AccelChipSelectPin.set_active(false);
        kBmi088AccelChipSelectPin.configure_as_output();
    }

    return init_spi_clock(ptr);
}

void init_gpio_pins() {
    kGpioHardwareDescriptors[0].configure_pioc_function();
    kGpioHardwareDescriptors[1].configure_pioc_function();
}

void init_user_button_and_switch_pins() {
    kUserHsFsSwitchPin.configure_controller();
    kUserHsFsSwitchPin.configure_ioc_function();
    kUserHsFsSwitchPin.configure_pad_control(
        IOC_PAD_PAD_CTL_PE_SET(1) | // Pull enable
        IOC_PAD_PAD_CTL_PS_SET(0) | // Pull select - Pull down
        IOC_PAD_PAD_CTL_HYS_SET(1)  // Enable Schmitt trigger
    );
    kUserHsFsSwitchPin.configure_as_input();
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B, gpio_bmi088_int_isr)
void gpio_bmi088_int_isr() {
    if (kBmi088GyroIntPin.check_clear_interrupt_flag()) {
        bmi088_gyro_dataready_irq_handler();
    }
    if (kBmi088AccelIntPin.check_clear_interrupt_flag()) {
        bmi088_accel_dataready_irq_handler();
    }
    gpio_irq_handler(GPIO_DI_GPIOB);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Y, gpio_y_isr)
void gpio_y_isr() { gpio_irq_handler(GPIO_DI_GPIOY); }

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

SDK_DECLARE_EXT_ISR_M(BOARD_UART_DBUS(IRQn_UART, ), uart_dbus_isr)
void uart_dbus_isr() { uart_dbus_irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_SPI_BMI088(IRQn_SPI, ), spi_isr)
void spi_isr() { spi_bmi088_irq_handler(); }

} // namespace librmcs::firmware::board
