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
#include <hpm_soc_irq.h>
#include <hpm_spi_regs.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/spi/spi.hpp"

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

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_D, bmi088_dataready_isr)
void bmi088_dataready_isr() {
    if (librmcs::firmware::board::kBmi088GyroIntPin.check_clear_interrupt_flag())
        librmcs::firmware::board::bmi088_gyro_dataready_irq_handler();
    if (librmcs::firmware::board::kBmi088AccelIntPin.check_clear_interrupt_flag())
        librmcs::firmware::board::bmi088_accel_dataready_irq_handler();
}

SDK_DECLARE_EXT_ISR_M(BOARD_SPI_BMI088(IRQn_SPI, ), spi_bmi088_isr)
void spi_bmi088_isr() { librmcs::firmware::board::spi_bmi088_irq_handler(); }
