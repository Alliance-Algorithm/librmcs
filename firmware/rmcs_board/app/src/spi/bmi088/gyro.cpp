#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"

#include "firmware/rmcs_board/app/src/spi/bmi088/service.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"

namespace librmcs::firmware::board {

void bmi088_gyro_dataready_irq_handler() {
    spi::bmi088::gyroscope->data_ready_callback(timer::Timer::timestamp_quarter_us());
    spi::bmi088::service_pending_reads();
}

} // namespace librmcs::firmware::board
