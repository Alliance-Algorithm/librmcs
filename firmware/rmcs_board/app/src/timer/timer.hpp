#pragma once

#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_soc.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/led/led.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::timer {

class Timer {
public:
    using Lazy = utility::Lazy<Timer>;

    static constexpr uint64_t kTimerFrequencyHz = 4'000'000U;

    static constexpr uint32_t kTickFrequencyHz = 1'000U;
    static constexpr uint64_t kTickPeriodTicks =
        (static_cast<uint64_t>(kTimerFrequencyHz) + (kTickFrequencyHz / 2U)) / kTickFrequencyHz;

    Timer() {
        core::utility::assert_always(clock_get_frequency(clock_mchtmr0) == kTimerFrequencyHz);

        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        next_tick_compare_value_ = timestamp64_quarter_us() + kTickPeriodTicks;
        mchtmr_set_compare_value(HPM_MCHTMR, next_tick_compare_value_);

        enable_mchtmr_irq();
    }

    void irq_handler() {
        tick_counter_ = tick_counter_ + 1;
        led::led->update(tick_counter_);

        do {
            next_tick_compare_value_ += kTickPeriodTicks;
        } while (next_tick_compare_value_ <= timestamp64_quarter_us());
        mchtmr_set_compare_value(HPM_MCHTMR, next_tick_compare_value_);
    }

    static uint32_t timestamp_quarter_us() {
        // Read only the lower 32 bits of MTIME with a single 32-bit load.
        // Assumes little-endian (riscv32), intentionally bypasses strict aliasing for efficiency.
        return *reinterpret_cast<volatile uint32_t*>(&HPM_MCHTMR->MTIME);
    }

    static uint64_t timestamp64_quarter_us() {
        // On riscv32, reading the 64-bit MTIME MMIO register compiles to two 32-bit loads.
        // Read high-low-high and retry on rollover so the returned 64-bit timestamp is stable.
        const volatile uint32_t* const mtime_words =
            reinterpret_cast<volatile uint32_t*>(&HPM_MCHTMR->MTIME);
        uint32_t hi1 = 0;
        uint32_t lo = 0;
        uint32_t hi2 = 0;

        do {
            hi1 = mtime_words[1];
            lo = mtime_words[0];
            hi2 = mtime_words[1];
        } while (hi1 != hi2);

        return (static_cast<uint64_t>(hi2) << 32U) | lo;
    }

private:
    uint64_t next_tick_compare_value_ = 0;
    uint32_t tick_counter_ = 0;
};

inline constinit Timer::Lazy timer;

} // namespace librmcs::firmware::timer
