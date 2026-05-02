#include "firmware/rmcs_board/app/src/spi/bmi088/temperature.hpp"

#include <cstdint>

#include <hpm_gptmr_drv.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "firmware/rmcs_board/app/src/spi/bmi088/service.hpp"

namespace librmcs::firmware::board {

namespace {

auto* const kTemperatureTimer = HPM_GPTMR1;
constexpr uint8_t kTemperatureTimerChannel = 0U;

} // namespace

SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR1, bmi088_temperature_timer_isr)
void bmi088_temperature_timer_isr() {
    if (!gptmr_check_status(kTemperatureTimer, GPTMR_CH_RLD_STAT_MASK(kTemperatureTimerChannel))) {
        return;
    }

    gptmr_clear_status(kTemperatureTimer, GPTMR_CH_RLD_STAT_MASK(kTemperatureTimerChannel));

    const uint32_t capture_timestamp_quarter_us =
        static_cast<uint32_t>(mchtmr_get_count(HPM_MCHTMR) / 6U);
    spi::bmi088::temperature->timer_callback(capture_timestamp_quarter_us);
    spi::bmi088::service_pending_reads();
}

} // namespace librmcs::firmware::board
