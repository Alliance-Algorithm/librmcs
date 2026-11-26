#include "board_app.hpp"

#include <cstdint>
#include <initializer_list>

#include <hpm_clock_drv.h>
#include <hpm_common.h> // IWYU pragma: keep
#include <hpm_gpio_drv.h>
#include <hpm_interrupt.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_mcan_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_ip_feature.h>
#include <hpm_soc_irq.h>
#include <hpm_spi_regs.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/spi/icm42688/icm42688.hpp"
#include "firmware/rmcs_board/app/src/spi/spi.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"
#include "rs485.hpp"

namespace librmcs::firmware::can {

void Can::init_pins(MCAN_Type* ptr) {
    if (ptr == HPM_MCAN0) {
        HPM_IOC->PAD[IOC_PAD_PD01].FUNC_CTL = IOC_PD01_FUNC_CTL_MCAN0_RXD;
        HPM_IOC->PAD[IOC_PAD_PD00].FUNC_CTL = IOC_PD00_FUNC_CTL_MCAN0_TXD;
    } else if (ptr == HPM_MCAN1) {
        HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PD04_FUNC_CTL_MCAN1_RXD;
        HPM_IOC->PAD[IOC_PAD_PD05].FUNC_CTL = IOC_PD05_FUNC_CTL_MCAN1_TXD;
    } else if (ptr == HPM_MCAN2) {
        HPM_IOC->PAD[IOC_PAD_PD09].FUNC_CTL = IOC_PD09_FUNC_CTL_MCAN2_RXD;
        HPM_IOC->PAD[IOC_PAD_PD08].FUNC_CTL = IOC_PD08_FUNC_CTL_MCAN2_TXD;
    } else if (ptr == HPM_MCAN3) {
        HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PC14_FUNC_CTL_MCAN3_RXD;
        HPM_IOC->PAD[IOC_PAD_PC15].FUNC_CTL = IOC_PC15_FUNC_CTL_MCAN3_TXD;
    }
}

} // namespace librmcs::firmware::can

uint32_t librmcs::firmware::spi::Spi::init_hardware(SPI_Type* ptr) {
    core::utility::assert_always(ptr == HPM_SPI3);
    clock_add_to_group(clock_spi3, 0);
    core::utility::assert_always(
        clock_set_source_divider(clock_spi3, clk_src_pll1_clk0, 5) == status_success);

    for (const auto* pin : {&board::kBmi088GyroChipSelectPin, &board::kBmi088AccelChipSelectPin}) {
        pin->configure_controller();
        pin->configure_ioc_function();
        pin->set_active(false);
        pin->configure_as_output();
    }

    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL =
        IOC_PC20_FUNC_CTL_SPI3_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC22].FUNC_CTL = IOC_PC22_FUNC_CTL_SPI3_MISO;
    HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PC23_FUNC_CTL_SPI3_MOSI;

    for (const auto* pin : {&board::kBmi088GyroIntPin, &board::kBmi088AccelIntPin}) {
        pin->configure_controller();
        pin->configure_ioc_function();
        pin->configure_as_input();
        pin->configure_interrupt(gpio_interrupt_trigger_edge_falling);
        pin->clear_interrupt_flag();
        pin->enable_interrupt();
    }
    intc_m_enable_irq_with_priority(IRQn_GPIO0_D, 1);

    return clock_get_frequency(clock_spi3);
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_D, imu_dataready_isr)
void imu_dataready_isr() {
    if (auto* icm = librmcs::firmware::spi::icm42688::sensor.try_get())
        icm->irq_handler();
    if (librmcs::firmware::board::kBmi088GyroIntPin.check_clear_interrupt_flag())
        librmcs::firmware::board::bmi088_gyro_dataready_irq_handler();
    if (librmcs::firmware::board::kBmi088AccelIntPin.check_clear_interrupt_flag())
        librmcs::firmware::board::bmi088_accel_dataready_irq_handler();
}

SDK_DECLARE_EXT_ISR_M(BOARD_SPI_BMI088(IRQn_SPI, ), spi_bmi088_isr)
void spi_bmi088_isr() { librmcs::firmware::board::spi_bmi088_irq_handler(); }

uint32_t librmcs::firmware::spi::icm42688::Icm42688::init_hardware(SPI_Type* spi) {
    core::utility::assert_always(spi == HPM_SPI2);
    constexpr uint32_t drive = IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(3);
    HPM_IOC->PAD[IOC_PAD_PC26].PAD_CTL = drive;
    HPM_IOC->PAD[IOC_PAD_PC28].PAD_CTL = drive;
    HPM_IOC->PAD[IOC_PAD_PC29].PAD_CTL = drive;
    HPM_IOC->PAD[IOC_PAD_PC26].FUNC_CTL =
        IOC_PC26_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC28].FUNC_CTL = IOC_PC28_FUNC_CTL_SPI2_MISO;
    HPM_IOC->PAD[IOC_PAD_PC29].FUNC_CTL = IOC_PC29_FUNC_CTL_SPI2_MOSI;
    clock_add_to_group(clock_spi2, 0);
    core::utility::assert_always(
        clock_set_source_divider(clock_spi2, clk_src_osc24m, 1) == status_success);
    return clock_get_frequency(clock_spi2);
}

namespace librmcs::firmware::uart {

uint32_t Uart::init_hardware(UART_Type* ptr) {
    constexpr uint32_t tx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    constexpr uint32_t rx_pad = tx_pad | IOC_PAD_PAD_CTL_HYS_SET(1);
    clock_name_t clock;
    uint32_t de_pad;
    uint32_t de_function;
    if (ptr == HPM_UART1) {
        clock = clock_uart1;
        de_pad = IOC_PAD_PC05;
        de_function = IOC_PC05_FUNC_CTL_UART1_DE;
        HPM_IOC->PAD[IOC_PAD_PC06].PAD_CTL = rx_pad;
        HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PC06_FUNC_CTL_UART1_RXD;
        HPM_IOC->PAD[IOC_PAD_PC07].PAD_CTL = tx_pad;
        HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PC07_FUNC_CTL_UART1_TXD;
    } else {
        core::utility::assert_always(ptr == HPM_UART4);
        clock = clock_uart4;
        de_pad = IOC_PAD_PC18;
        de_function = IOC_PC18_FUNC_CTL_UART4_DE;
        HPM_IOC->PAD[IOC_PAD_PC17].PAD_CTL = rx_pad;
        HPM_IOC->PAD[IOC_PAD_PC17].FUNC_CTL = IOC_PC17_FUNC_CTL_UART4_RXD;
        HPM_IOC->PAD[IOC_PAD_PC16].PAD_CTL = tx_pad;
        HPM_IOC->PAD[IOC_PAD_PC16].FUNC_CTL = IOC_PC16_FUNC_CTL_UART4_TXD;
    }

    clock_add_to_group(clock, 0);
    core::utility::assert_always(
        clock_set_source_divider(clock, clk_src_osc24m, 1) == status_success);
    // RE# and DE are tied: low receives, high transmits. Let UART own the
    // turnaround, including the last stop bit; DMA completion is too early.
    static_assert(HPM_IP_FEATURE_UART_DE_DELAY == 1);
    constexpr uint8_t de_delay_divided_clock_ticks = 2;
    uart_set_de_delay_before_start_bit(ptr, de_delay_divided_clock_ticks);
    uart_set_de_delay_after_stop_bit(ptr, de_delay_divided_clock_ticks);
    HPM_IOC->PAD[de_pad].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1);
    HPM_IOC->PAD[de_pad].FUNC_CTL = de_function;
    return clock_get_frequency(clock);
}

} // namespace librmcs::firmware::uart

namespace librmcs::firmware::board {

SDK_DECLARE_EXT_ISR_M(BOARD_RS4850(IRQn_UART, ), rs4850_isr)
void rs4850_isr() { rs485::ports[0]->irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_RS4851(IRQn_UART, ), rs4851_isr)
void rs4851_isr() { rs485::ports[1]->irq_handler(); }

} // namespace librmcs::firmware::board
