#include "firmware/rmcs_board/src/uart/uart.hpp"

#include <hpm_soc.h>
#include <hpm_soc_irq.h>

namespace librmcs::firmware::uart {

SDK_DECLARE_EXT_ISR_M(IRQn_UART3, uart3_isr)
void uart3_isr() { uart3->isr(); }

} // namespace librmcs::firmware::uart
