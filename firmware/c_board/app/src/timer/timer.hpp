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
    using FullDuration = std::chrono::duration<uint64_t, TickPeriod>;

    using TimePoint = std::chrono::time_point<uint32_t, Duration>;
    using FullTimePoint = std::chrono::time_point<uint64_t, FullDuration>;

    // Keep the true-window at least half-cycle for stateless expiration checks.
    static constexpr uint64_t kMaxDelay32Ticks = uint64_t{1} << 31;
    static constexpr uint64_t kMaxDelay48Ticks = uint64_t{1} << 47;

    static constexpr uint64_t kFullCounterMask = 0xFFFFFFFFFFFFULL;

    static constexpr TIM_HandleTypeDef* kTimerLow = &htim2;
    static constexpr TIM_HandleTypeDef* kTimerHigh = &htim9;

    Timer()
        : timer_counter_high_(kTimerHigh->Instance->CNT)
        , timer_counter_low_(kTimerLow->Instance->CNT) {
        core::utility::assert_always(
            HAL_TIM_Base_Start(kTimerHigh) == HAL_OK && HAL_TIM_Base_Start(kTimerLow) == HAL_OK);
    }

    TimePoint time_point() const { return TimePoint{Duration{timer_counter_low_}}; }

    FullTimePoint full_time_point() const {
        uint32_t hi1, hi2, lo;
        do {
            hi1 = timer_counter_high_;
            lo = timer_counter_low_;
            hi2 = timer_counter_high_;
        } while (hi1 != hi2);

        return FullTimePoint{FullDuration{(uint64_t{hi2} << 32) | lo}};
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] bool
        delay_async(TimePoint start_point, std::chrono::duration<Rep, Period> delay) {
        const uint32_t delay_ticks = duration_to_ticks_checked_32(delay);
        const uint32_t start_ticks = start_point.time_since_epoch().count();
        const uint32_t now_ticks = time_point().time_since_epoch().count();
        return static_cast<uint32_t>(now_ticks - start_ticks) >= delay_ticks;
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] bool
        delay_async(FullTimePoint start_point, std::chrono::duration<Rep, Period> delay) {
        const uint64_t delay_ticks = duration_to_ticks_checked_48(delay);
        const uint64_t start_ticks = start_point.time_since_epoch().count() & kFullCounterMask;
        const uint64_t now_ticks = full_time_point().time_since_epoch().count() & kFullCounterMask;
        const uint64_t elapsed_ticks = (now_ticks - start_ticks) & kFullCounterMask;
        return elapsed_ticks >= delay_ticks;
    }

    template <std::integral Rep, typename Period>
    void delay(std::chrono::duration<Rep, Period> delay) {
        using InputDuration = std::chrono::duration<uint64_t, Period>;
        InputDuration remaining{count_to_u64_checked(delay.count())};
        if (!remaining.count()) [[unlikely]]
            return;

        constexpr FullDuration max_segment_ticks{kMaxDelay48Ticks};
        constexpr InputDuration max_segment =
            std::chrono::duration_cast<InputDuration>(max_segment_ticks);
        static_assert(
            max_segment.count() > 0, "The unit is too large, please choose a smaller unit");

        while (remaining > max_segment) {
            remaining -= max_segment;
            delay_basic(max_segment_ticks);
        }

        delay_basic(FullDuration{duration_to_ticks_checked_ceil<kMaxDelay48Ticks>(remaining)});
    }

private:
    void delay_basic(FullDuration delay) {
        const FullTimePoint start = full_time_point();
        while (!delay_async(start, delay))
            ;
    }

    template <std::integral Rep>
    [[nodiscard]] static uint64_t count_to_u64_checked(Rep count) {
        if constexpr (std::is_signed_v<Rep>)
            core::utility::assert_always(count >= 0);

        if constexpr (sizeof(Rep) > sizeof(uint64_t)) {
            core::utility::assert_always(
                count <= static_cast<Rep>(std::numeric_limits<uint64_t>::max()));
        }
        return static_cast<uint64_t>(count);
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] static uint32_t
        duration_to_ticks_checked_32(std::chrono::duration<Rep, Period> duration) {
        constexpr uint64_t max_ticks = kMaxDelay32Ticks;
        return static_cast<uint32_t>(duration_to_ticks_checked<max_ticks>(duration));
    }

    template <std::integral Rep, typename Period>
    [[nodiscard]] static uint64_t
        duration_to_ticks_checked_48(std::chrono::duration<Rep, Period> duration) {
        return duration_to_ticks_checked<kMaxDelay48Ticks>(duration);
    }

    template <uint64_t max_ticks, std::integral Rep, typename Period>
    [[nodiscard]] static uint64_t
        duration_to_ticks_checked_ceil(std::chrono::duration<Rep, Period> duration) {
        const uint64_t floor_ticks = duration_to_ticks_checked<max_ticks>(duration);

        const uint64_t count = count_to_u64_checked(duration.count());
        using InputDuration = std::chrono::duration<uint64_t, Period>;

        const InputDuration duration_u64{count};
        const InputDuration floor_duration =
            std::chrono::duration_cast<InputDuration>(FullDuration{floor_ticks});
        if (floor_duration < duration_u64) {
            core::utility::assert_always(floor_ticks < max_ticks);
            return floor_ticks + 1;
        }
        return floor_ticks;
    }

    template <uint64_t max_ticks, std::integral Rep, typename Period>
    [[nodiscard]] static uint64_t
        duration_to_ticks_checked(std::chrono::duration<Rep, Period> duration) {
        static_assert(Period::num > 0 && Period::den > 0);

        const uint64_t count = count_to_u64_checked(duration.count());
        using InputDuration = std::chrono::duration<uint64_t, Period>;

        const InputDuration duration_u64{count};
        constexpr FullDuration max_duration{max_ticks};
        const InputDuration max_input_duration =
            std::chrono::duration_cast<InputDuration>(max_duration);

        core::utility::assert_always(duration_u64 <= max_input_duration);
        const uint64_t ticks = std::chrono::duration_cast<FullDuration>(duration_u64).count();

        core::utility::assert_always(ticks <= max_ticks);
        return ticks;
    }

    const volatile uint32_t& timer_counter_high_;
    const volatile uint32_t& timer_counter_low_;
};

inline constinit Timer::Lazy timer;

} // namespace librmcs::firmware::timer
