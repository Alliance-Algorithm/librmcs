#include "firmware/rmcs_board/app/src/timer/tick.hpp"

#include "board_app.hpp"

namespace librmcs::firmware::board {

void tick_clock_irq_handler() { timer::tick->irq_handler(); }

} // namespace librmcs::firmware::board
