#include <hpm_clock_drv.h>
#include <hpm_common.h> // IWYU pragma: keep
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_mcan_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"
#include "firmware/rmcs_board/boards/uart_ports.hpp"

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

namespace librmcs::firmware::uart {

uint32_t Uart::init_hardware(UART_Type* ptr) {
    constexpr uint32_t tx_pad = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    constexpr uint32_t rx_pad = tx_pad | IOC_PAD_PAD_CTL_HYS_SET(1);
    clock_name_t clock;
    if (ptr == HPM_UART6) {
        clock = clock_uart6;
        HPM_IOC->PAD[IOC_PAD_PC24].PAD_CTL = tx_pad;
        HPM_IOC->PAD[IOC_PAD_PC24].FUNC_CTL = IOC_PC24_FUNC_CTL_UART6_TXD;
        HPM_IOC->PAD[IOC_PAD_PC25].PAD_CTL = rx_pad;
        HPM_IOC->PAD[IOC_PAD_PC25].FUNC_CTL = IOC_PC25_FUNC_CTL_UART6_RXD;
    } else if (ptr == HPM_UART0) {
        clock = clock_uart0;
        HPM_IOC->PAD[IOC_PAD_PA00].PAD_CTL = tx_pad;
        HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_UART0_TXD;
        HPM_IOC->PAD[IOC_PAD_PA01].PAD_CTL = rx_pad;
        HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_UART0_RXD;
    } else if (ptr == HPM_UART7) {
        clock = clock_uart7;
        HPM_IOC->PAD[IOC_PAD_PC31].PAD_CTL = tx_pad;
        HPM_IOC->PAD[IOC_PAD_PC31].FUNC_CTL = IOC_PC31_FUNC_CTL_UART7_TXD;
        HPM_IOC->PAD[IOC_PAD_PC30].PAD_CTL = rx_pad;
        HPM_IOC->PAD[IOC_PAD_PC30].FUNC_CTL = IOC_PC30_FUNC_CTL_UART7_RXD;
    } else {
        core::utility::assert_always(ptr == HPM_UART2);
        clock = clock_uart2;
        HPM_IOC->PAD[IOC_PAD_PC09].PAD_CTL = rx_pad;
        HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PC09_FUNC_CTL_UART2_RXD;
    }

    clock_add_to_group(clock, 0);
    core::utility::assert_always(
        clock_set_source_divider(clock, clk_src_osc24m, 1) == status_success);
    return clock_get_frequency(clock);
}

} // namespace librmcs::firmware::uart

namespace librmcs::firmware::board {

SDK_DECLARE_EXT_ISR_M(BOARD_UART0(IRQn_UART, ), uart0_isr)
void uart0_isr() { uart_array[0]->irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART1(IRQn_UART, ), uart1_isr)
void uart1_isr() { uart_array[1]->irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART2(IRQn_UART, ), uart2_isr)
void uart2_isr() { uart_array[2]->irq_handler(); }

SDK_DECLARE_EXT_ISR_M(BOARD_UART_DBUS(IRQn_UART, ), uart_dbus_isr)
void uart_dbus_isr() { uart_dbus->irq_handler(); }

} // namespace librmcs::firmware::board
