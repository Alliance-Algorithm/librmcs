#include "firmware/rmcs_board/src/uart/uart.hpp"

#include <board.h>
#include <hpm_uart_drv.h>

namespace librmcs::firmware::uart {

SDK_DECLARE_EXT_ISR_M(IRQn_UART3, uart3_isr)
void uart3_isr() { uart3->isr(); }

} // namespace librmcs::firmware::uart
