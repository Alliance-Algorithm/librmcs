#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"

namespace librmcs::firmware::board {

void bmi088_accel_dataready_irq_handler() { spi::bmi088::accelerometer->data_ready_callback(); }

} // namespace librmcs::firmware::board
