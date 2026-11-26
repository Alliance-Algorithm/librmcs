#include "firmware/rmcs_board/app/src/timer/timer.hpp"

// hpm_interrupt.h uses the SoC definitions provided by hpm_soc.h.
// clang-format off
#include <hpm_soc.h> // IWYU pragma: keep
#include <hpm_interrupt.h>
// clang-format on

SDK_DECLARE_MCHTMR_ISR(tick_clock_isr)
void tick_clock_isr() { librmcs::firmware::timer::timer->irq_handler(); }
