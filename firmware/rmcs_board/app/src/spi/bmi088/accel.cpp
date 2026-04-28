#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"

#include <cstdint>

#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>

namespace librmcs::firmware::board {

void bmi088_accel_dataready_irq_handler() {
    const uint32_t capture_timestamp_quarter_us =
        static_cast<uint32_t>(mchtmr_get_count(HPM_MCHTMR) / 6U);
    spi::bmi088::accelerometer->data_ready_callback(capture_timestamp_quarter_us);
}

} // namespace librmcs::firmware::board
