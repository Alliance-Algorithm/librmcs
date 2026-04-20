#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <librmcs/spec/gpio.hpp>

namespace librmcs::spec::c_board {

namespace internal {
class GpioDescriptors;
}

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class GpioDescriptor : public spec::GpioDescriptor {
    friend internal::GpioDescriptors;
    constexpr GpioDescriptor(uint8_t channel_index, GpioCapability capability_mask)
        : spec::GpioDescriptor(channel_index, capability_mask) {}

public:
    GpioDescriptor(const GpioDescriptor&) = delete;
    GpioDescriptor& operator=(const GpioDescriptor&) = delete;
    GpioDescriptor(GpioDescriptor&&) = delete;
    GpioDescriptor& operator=(GpioDescriptor&&) = delete;

    [[nodiscard]] constexpr bool operator==(const GpioDescriptor& other) const noexcept {
        return channel_index == other.channel_index;
    }
};

namespace internal {
class GpioDescriptors {
    static constexpr GpioDescriptor kArray[]{
        {0,                                          kAllGpioCapabilities},
        {1,                                          kAllGpioCapabilities},
        {2,                                          kAllGpioCapabilities},
        {3,                                          kAllGpioCapabilities},
        {4,                                          kAllGpioCapabilities},
        {5, kAllGpioCapabilities & ~GpioCapability::kDigitalReadInterrupt},
        {6,                                          kAllGpioCapabilities}
    };
    static_assert(channel_indices_match_indices(kArray));

public:
    constexpr GpioDescriptors() = default;

    static constexpr std::size_t size() noexcept { return std::size(kArray); }

    static constexpr const GpioDescriptor& operator[](std::size_t channel_index) noexcept {
        return kArray[channel_index];
    }

    static constexpr const GpioDescriptor* begin() noexcept { return std::begin(kArray); }

    static constexpr const GpioDescriptor* end() noexcept { return std::end(kArray); }

    static constexpr const GpioDescriptor& kPwm1 = kArray[0];
    static constexpr const GpioDescriptor& kPwm2 = kArray[1];
    static constexpr const GpioDescriptor& kPwm3 = kArray[2];
    static constexpr const GpioDescriptor& kPwm4 = kArray[3];
    static constexpr const GpioDescriptor& kPwm5 = kArray[4];
    static constexpr const GpioDescriptor& kPwm6 = kArray[5];
    static constexpr const GpioDescriptor& kPwm7 = kArray[6];
};
} // namespace internal

inline constexpr internal::GpioDescriptors kGpioDescriptors{};

} // namespace librmcs::spec::c_board
