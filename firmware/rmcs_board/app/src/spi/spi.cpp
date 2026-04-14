#include "firmware/rmcs_board/app/src/spi/spi.hpp"

#include "board_app.hpp"

namespace librmcs::firmware::board {

void spi_bmi088_irq_handler() { spi::spi_bmi088->irq_handler(); }

} // namespace librmcs::firmware::board
