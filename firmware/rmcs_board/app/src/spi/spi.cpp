#include "firmware/rmcs_board/app/src/spi/spi.hpp"

#include "board_app.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/service.hpp"

namespace librmcs::firmware::board {

void spi_bmi088_irq_handler() {
    spi::spi_bmi088->irq_handler();
    spi::bmi088::service_pending_reads();
}

} // namespace librmcs::firmware::board
