#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"

namespace librmcs::firmware::board {

void bmi088_gyro_dataready_irq_handler() { spi::bmi088::gyroscope->data_ready_callback(); }

} // namespace librmcs::firmware::board
