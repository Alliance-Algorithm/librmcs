#pragma once

#include <cstdint>

#include <hpm_gptmr_drv.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "board_app.hpp"
#include "firmware/rmcs_board/app/src/led/led.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::timer {

class Tick {
public:
    using Lazy = utility::Lazy<Tick>;

    Tick() {
        const uint32_t freq = board::init_tick_clock();

        gptmr_channel_config_t config;
        gptmr_channel_get_default_config(BOARD_TICK_CLOCK, &config);
        config.reload = (freq + 500) / 1000U; // 1kHz
        gptmr_channel_config(BOARD_TICK_CLOCK, 0, &config, false);

        gptmr_enable_irq(BOARD_TICK_CLOCK, GPTMR_CH_RLD_IRQ_MASK(0));
        intc_m_enable_irq_with_priority(IRQn_GPTMR1, 1);

        gptmr_start_counter(BOARD_TICK_CLOCK, 0);
    }

    void irq_handler() {
        if (gptmr_check_status(BOARD_TICK_CLOCK, GPTMR_CH_RLD_STAT_MASK(0))) {
            gptmr_clear_status(BOARD_TICK_CLOCK, GPTMR_CH_RLD_STAT_MASK(0));
            tick_counter_ = tick_counter_ + 1;
            led::led->update(tick_counter_);
        }
    }

private:
    uint32_t tick_counter_ = 0;
};

inline constinit Tick::Lazy tick;

} // namespace librmcs::firmware::timer
