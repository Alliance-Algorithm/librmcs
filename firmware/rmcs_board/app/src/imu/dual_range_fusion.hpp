#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace librmcs::firmware::imu {

class DualRangeFusion final {
public:
    using Vector = std::array<float, 3>;

    struct Sample {
        Vector value;        // Calibrated physical units in the common body frame.
        float peak_fraction; // Native-axis peak / full scale; use 1 or above for clipped data.
        uint32_t timestamp_quarter_us;
    };

    enum class Source : uint8_t { kPrecision, kBlend, kWide };

    struct Result {
        Vector value;
        uint32_t timestamp_quarter_us;
        Source source;
    };

    struct Timing {
        uint32_t max_age_quarter_us;
        uint32_t max_skew_quarter_us;
    };

    // Apply independently to gyroscope and accelerometer samples on the same capture clock.
    static constexpr std::optional<Result> fuse(
        const std::optional<Sample>& precision_sample, const std::optional<Sample>& wide_sample,
        uint32_t now_quarter_us, Timing timing) noexcept {
        const auto* precision =
            usable_sample(precision_sample, now_quarter_us, timing.max_age_quarter_us);
        const auto* wide = usable_sample(wide_sample, now_quarter_us, timing.max_age_quarter_us);
        if (!precision) {
            if (wide)
                return select(*wide, Source::kWide);
            return std::nullopt;
        }
        if (!wide)
            return select(*precision, Source::kPrecision);

        const uint32_t precision_age = now_quarter_us - precision->timestamp_quarter_us;
        const uint32_t wide_age = now_quarter_us - wide->timestamp_quarter_us;
        const bool wide_newer = wide_age < precision_age;
        const uint32_t skew = wide_newer ? precision_age - wide_age : wide_age - precision_age;
        if (skew > timing.max_skew_quarter_us) {
            if (wide_newer)
                return select(*wide, Source::kWide);
            return select(*precision, Source::kPrecision);
        }

        if (precision->peak_fraction <= kBlendStart)
            return select(*precision, Source::kPrecision);
        if (precision->peak_fraction >= kBlendEnd)
            return select(*wide, Source::kWide);

        const float weight = (precision->peak_fraction - kBlendStart) / (kBlendEnd - kBlendStart);
        Result result = select(wide_newer ? *wide : *precision, Source::kBlend);
        for (std::size_t axis = 0; axis < result.value.size(); ++axis)
            result.value[axis] = std::lerp(precision->value[axis], wide->value[axis], weight);
        return result;
    }

private:
    static constexpr const Sample* usable_sample(
        const std::optional<Sample>& sample, uint32_t now, uint32_t max_age) noexcept {
        if (!sample || !std::isfinite(sample->peak_fraction) || sample->peak_fraction < 0.0F
            || sample->peak_fraction >= 1.0F)
            return nullptr;
        // Unsigned subtraction handles capture-clock wraparound.
        const uint32_t age = now - sample->timestamp_quarter_us;
        if (age > max_age || age >= kHalfClockPeriod
            || !std::ranges::all_of(sample->value, [](float axis) { return std::isfinite(axis); }))
            return nullptr;
        return &*sample;
    }

    static constexpr Result select(const Sample& sample, Source source) noexcept {
        return {
            .value = sample.value,
            .timestamp_quarter_us = sample.timestamp_quarter_us,
            .source = source,
        };
    }

    static constexpr uint32_t kHalfClockPeriod = uint32_t{1} << 31U;
    static constexpr float kBlendStart = 0.70F;
    static constexpr float kBlendEnd = 0.90F;
};

} // namespace librmcs::firmware::imu
