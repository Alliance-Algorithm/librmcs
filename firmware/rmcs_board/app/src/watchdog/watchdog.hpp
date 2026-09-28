#pragma once

#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_ewdg_drv.h>
#include <hpm_soc.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

// Timeout override for targets with longer IRQ-disabled sections (e.g. bootloader
// flash erase, worst case ~400ms plus program time).
#ifndef LIBRMCS_WATCHDOG_TIMEOUT_US
# define LIBRMCS_WATCHDOG_TIMEOUT_US (500'000U)
#endif

namespace librmcs::firmware::watchdog {

class Watchdog {
public:
    using Lazy = utility::Lazy<Watchdog>;

    Watchdog() {
        clock_add_to_group(clock_watchdog0, 0);

        ewdg_config_t config;
        ewdg_get_default_config(HPM_EWDG0, &config);

        config.enable_watchdog = true;
        config.int_rst_config.enable_timeout_reset = true;
        config.ctrl_config.use_lowlevel_timeout = false;
        config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;
        config.ctrl_config.keep_running_in_debug_mode = false;
        config.ctrl_config.timeout_reset_us = kTimeoutUs;
        config.cnt_src_freq = kClockFrequencyHz;

        core::utility::assert_always(ewdg_init(HPM_EWDG0, &config) == status_success);
    }

    static void feed() { core::utility::assert_always(ewdg_refresh(HPM_EWDG0) == status_success); }

private:
    static constexpr uint32_t kTimeoutUs = LIBRMCS_WATCHDOG_TIMEOUT_US;
    static constexpr uint32_t kClockFrequencyHz = 32'768U;
};

inline constinit Watchdog::Lazy watchdog;

} // namespace librmcs::firmware::watchdog
