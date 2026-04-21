#pragma once

#include <chrono>
#include <concepts>
#include <cstdint>
#include <limits>
#include <ratio>
#include <type_traits>

#include <main.h>
#include <tim.h>

#include "core/src/utility/assert.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::timer {

class Timer {
public:
    using Lazy = utility::Lazy<Timer>;

    static constexpr uint32_t kClockFrequency = 168'000'000 / 2 / 21;
    using TickPeriod = std::ratio<1, kClockFrequency>;

    // 1/4 us
    using Duration = std::chrono::duration<uint32_t, TickPeriod>;
    using Duration48 = std::chrono::duration<uint64_t, TickPeriod>;

    using TimePoint = std::chrono::time_point<uint32_t, Duration>;
    using TimePoint48 = std::chrono::time_point<uint64_t, Duration48>;

    // Keep the true-window at least half-cycle for stateless expiration checks.
    static constexpr uint32_t kMaxDurationTicks = uint32_t{1} << 31;
    static constexpr uint64_t kMaxDuration48Ticks = uint64_t{1} << 47;

    static constexpr uint64_t kCounter48Mask = 0xFFFFFFFFFFFFULL;

    static constexpr TIM_HandleTypeDef* kTimerLow = &htim2;
    static constexpr TIM_HandleTypeDef* kTimerHigh = &htim9;

    Timer()
        : timer_counter_high_(kTimerHigh->Instance->CNT)
        , timer_counter_low_(kTimerLow->Instance->CNT) {
        core::utility::assert_always(
            HAL_TIM_Base_Start(kTimerHigh) == HAL_OK && HAL_TIM_Base_Start(kTimerLow) == HAL_OK);
    }

    TimePoint timepoint() const { return TimePoint{Duration{timer_counter_low_}}; }

    TimePoint48 timepoint48() const {
        uint32_t hi1, hi2, lo;
        do {
            hi1 = timer_counter_high_;
            lo = timer_counter_low_;
            hi2 = timer_counter_high_;
        } while (hi1 != hi2);

        return TimePoint48{Duration48{(uint64_t{hi2} << 32) | lo}};
    }

    [[nodiscard]] bool check_expired(TimePoint start_point, Duration delay) const {
        core::utility::assert_debug(delay.count() <= kMaxDurationTicks);

        const uint32_t start_ticks = start_point.time_since_epoch().count();
        const uint32_t now_ticks = timepoint().time_since_epoch().count();
        const Duration elapsed_duration{static_cast<uint32_t>(now_ticks - start_ticks)};
        return elapsed_duration >= delay;
    }

    [[nodiscard]] bool check_reached(TimePoint deadline) const {
        const uint32_t deadline_ticks = deadline.time_since_epoch().count();
        const uint32_t now_ticks = timepoint().time_since_epoch().count();
        const uint32_t elapsed_ticks = now_ticks - deadline_ticks;
        return elapsed_ticks < kMaxDurationTicks;
    }

    [[nodiscard]] bool check_expired(TimePoint48 start_point, Duration48 delay) const {
        core::utility::assert_debug(delay.count() <= kMaxDuration48Ticks);

        const uint64_t start_ticks = start_point.time_since_epoch().count() & kCounter48Mask;
        const uint64_t now_ticks = timepoint48().time_since_epoch().count() & kCounter48Mask;
        const Duration48 elapsed_duration{(now_ticks - start_ticks) & kCounter48Mask};
        return elapsed_duration >= delay;
    }

    void spin_wait(Duration48 delay) const {
        core::utility::assert_debug(delay.count() <= kMaxDuration48Ticks);

        const TimePoint48 start = timepoint48();
        while (!check_expired(start, delay))
            ;
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] static constexpr Duration
        to_duration_checked(std::chrono::duration<Rep, Period> duration) {
        static_assert(Period::num > 0 && Period::den > 0);

        const uint64_t count = count_to_u64_checked(duration.count());
        using InputDuration = std::chrono::duration<uint64_t, Period>;
        const InputDuration duration_u64{count};

        constexpr Duration max_duration{kMaxDurationTicks};
        const InputDuration max_input_duration =
            std::chrono::duration_cast<InputDuration>(max_duration);

        core::utility::assert_debug(duration_u64 <= max_input_duration);
        const Duration delay_duration = std::chrono::ceil<Duration>(duration_u64);

        core::utility::assert_debug(delay_duration.count() <= kMaxDurationTicks);
        return delay_duration;
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] static constexpr Duration48
        to_duration48_checked(std::chrono::duration<Rep, Period> duration) {
        static_assert(Period::num > 0 && Period::den > 0);

        const uint64_t count = count_to_u64_checked(duration.count());
        using InputDuration = std::chrono::duration<uint64_t, Period>;
        const InputDuration duration_u64{count};

        constexpr Duration48 max_duration{kMaxDuration48Ticks};
        const InputDuration max_input_duration =
            std::chrono::duration_cast<InputDuration>(max_duration);

        core::utility::assert_debug(duration_u64 <= max_input_duration);
        const Duration48 delay_duration = std::chrono::ceil<Duration48>(duration_u64);

        core::utility::assert_debug(delay_duration.count() <= kMaxDuration48Ticks);
        return delay_duration;
    }

private:
    template <std::integral Rep>
    [[nodiscard]] static uint64_t count_to_u64_checked(Rep count) {
        if constexpr (std::is_signed_v<Rep>)
            core::utility::assert_debug(count >= 0);

        if constexpr (sizeof(Rep) > sizeof(uint64_t)) {
            core::utility::assert_debug(
                count <= static_cast<Rep>(std::numeric_limits<uint64_t>::max()));
        }
        return static_cast<uint64_t>(count);
    }

    const volatile uint32_t& timer_counter_high_;
    const volatile uint32_t& timer_counter_low_;
};

inline constinit Timer::Lazy timer;

} // namespace librmcs::firmware::timer
