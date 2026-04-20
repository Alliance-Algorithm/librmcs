#pragma once

#include <cstddef>
#include <cstdint>

namespace librmcs::spec {

enum class GpioCapability : std::uint8_t {
    kNone = 0,
    kDigitalWrite = 1U << 0,
    kAnalogWrite = 1U << 1,
    kDigitalReadOnce = 1U << 2,
    kDigitalReadPeriodic = 1U << 3,
    kDigitalReadInterrupt = 1U << 4,
    kPullUp = 1U << 5,
    kPullDown = 1U << 6,
};

constexpr GpioCapability operator|(GpioCapability a, GpioCapability b) noexcept {
    return static_cast<GpioCapability>(static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b));
}

constexpr GpioCapability operator&(GpioCapability a, GpioCapability b) noexcept {
    return static_cast<GpioCapability>(static_cast<std::uint8_t>(a) & static_cast<std::uint8_t>(b));
}

constexpr GpioCapability operator~(GpioCapability a) noexcept {
    return static_cast<GpioCapability>(~static_cast<std::uint8_t>(a));
}

inline constexpr GpioCapability kAllGpioCapabilities =
    GpioCapability::kDigitalWrite | GpioCapability::kAnalogWrite | GpioCapability::kDigitalReadOnce
    | GpioCapability::kDigitalReadPeriodic | GpioCapability::kDigitalReadInterrupt
    | GpioCapability::kPullUp | GpioCapability::kPullDown;

struct GpioDescriptor {
    std::uint8_t channel_index;
    GpioCapability capability_mask = GpioCapability::kNone;

    [[nodiscard]] constexpr bool supports(GpioCapability capability) const noexcept {
        return (capability_mask & capability) == capability;
    }
};

template <typename Descriptor, std::size_t n>
[[nodiscard]] consteval bool
    channel_indices_match_indices(const Descriptor (&descriptors)[n]) noexcept {
    for (std::size_t index = 0; index < n; ++index) {
        if (descriptors[index].channel_index != index)
            return false;
    }
    return true;
}

} // namespace librmcs::spec
