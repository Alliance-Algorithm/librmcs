#include "firmware/rmcs_board/app/src/imu/dual_range_fusion.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace librmcs::firmware::imu::test {

class DualRangeFusionTest final {
    using Fusion = DualRangeFusion;
    using Source = Fusion::Source;

    static constexpr uint32_t kNow = 100;
    static constexpr Fusion::Timing kTiming{.max_age_quarter_us = 20, .max_skew_quarter_us = 5};
    static constexpr Fusion::Sample kWide{
        .value = {84, -20, 12},
        .peak_fraction = 0.21F,
        .timestamp_quarter_us = 99,
    };
    static constexpr Fusion::Sample kPrecision{
        .value = {80, -10, 8},
        .peak_fraction = 0.80F,
        .timestamp_quarter_us = 97,
    };

public:
    static constexpr bool blends_in_overlap() {
        const auto result = Fusion::fuse(kPrecision, kWide, kNow, kTiming);
        return result && result->source == Source::kBlend && result->timestamp_quarter_us == 99
            && std::abs(result->value[0] - 82.0F) < 0.001F
            && std::abs(result->value[1] - (-15.0F)) < 0.001F
            && std::abs(result->value[2] - 10.0F) < 0.001F;
    }

    static constexpr bool selects_at_boundaries() {
        auto precision = kPrecision;
        precision.peak_fraction = 0.70F;
        const auto low = Fusion::fuse(precision, kWide, kNow, kTiming);
        precision.peak_fraction = 0.90F;
        const auto high = Fusion::fuse(precision, kWide, kNow, kTiming);
        precision.peak_fraction = 1.0F;
        const auto clipped = Fusion::fuse(precision, kWide, kNow, kTiming);
        return low && low->source == Source::kPrecision && low->value == precision.value && high
            && high->source == Source::kWide && high->value == kWide.value && clipped
            && clipped->source == Source::kWide && !Fusion::fuse(precision, {}, kNow, kTiming);
    }

    static constexpr bool handles_missing_or_invalid_data() {
        auto wide = kWide;
        wide.peak_fraction = 1.0F;
        const auto fallback = Fusion::fuse(kPrecision, wide, kNow, kTiming);
        auto precision = kPrecision;
        precision.value[0] = std::numeric_limits<float>::infinity();
        const auto invalid = Fusion::fuse(precision, kWide, kNow, kTiming);
        precision.peak_fraction = 1.0F;
        return fallback && fallback->source == Source::kPrecision && invalid
            && invalid->source == Source::kWide && !Fusion::fuse(precision, wide, kNow, kTiming)
            && !Fusion::fuse({}, {}, kNow, kTiming);
    }

    static constexpr bool checks_time() {
        auto precision = kPrecision;
        auto wide = kWide;
        precision.timestamp_quarter_us = 91;
        const auto mismatched = Fusion::fuse(precision, wide, kNow, kTiming);
        wide.timestamp_quarter_us = 79;
        const auto stale = Fusion::fuse(precision, wide, kNow, kTiming);
        wide.timestamp_quarter_us = 101;
        const auto future = Fusion::fuse(precision, wide, kNow, kTiming);
        precision.timestamp_quarter_us = std::numeric_limits<uint32_t>::max();
        wide.timestamp_quarter_us = 0;
        const auto wrapped = Fusion::fuse(precision, wide, 1, kTiming);
        return mismatched && mismatched->source == Source::kWide && stale
            && stale->source == Source::kPrecision && future && future->source == Source::kPrecision
            && wrapped && wrapped->source == Source::kBlend && wrapped->timestamp_quarter_us == 0;
    }
};

static_assert(DualRangeFusionTest::blends_in_overlap());
static_assert(DualRangeFusionTest::selects_at_boundaries());
static_assert(DualRangeFusionTest::handles_missing_or_invalid_data());
static_assert(DualRangeFusionTest::checks_time());

} // namespace librmcs::firmware::imu::test
