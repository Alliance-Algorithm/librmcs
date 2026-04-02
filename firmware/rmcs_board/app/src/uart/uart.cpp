#include "firmware/rmcs_board/app/src/uart/uart.hpp"

#include <hpm_soc.h>
#include <hpm_soc_irq.h>

namespace librmcs::firmware::uart {

SDK_DECLARE_EXT_ISR_M(IRQn_UART0, uart0_isr)
void uart0_isr() { uart0->isr(); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART3, uart1_isr)
void uart1_isr() { uart1->isr(); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART5, uart2_isr)
void uart2_isr() { uart2->isr(); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART1, uart3_isr)
void uart3_isr() { uart3->isr(); }

SDK_DECLARE_EXT_ISR_M(IRQn_UART2, uart_dbus_isr)
void uart_dbus_isr() { uart_dbus->isr(); }

} // namespace librmcs::firmware::uart
