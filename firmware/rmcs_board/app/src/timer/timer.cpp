#include "firmware/rmcs_board/app/src/timer/timer.hpp"

#include <hpm_soc.h>

SDK_DECLARE_MCHTMR_ISR(tick_clock_isr)
void tick_clock_isr() { librmcs::firmware::timer::timer->irq_handler(); }
