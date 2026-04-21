#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <librmcs/spec/gpio.hpp>

namespace librmcs::spec::rmcs_board_lite {

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
        {0, kDigitalCapabilities},
        {1, kDigitalCapabilities},
        {2, kDigitalCapabilities},
        {3, kDigitalCapabilities},
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

    static constexpr const GpioDescriptor& kUart0Rx = kArray[0];
    static constexpr const GpioDescriptor& kUart0Tx = kArray[1];
    static constexpr const GpioDescriptor& kUart1Rx = kArray[2];
    static constexpr const GpioDescriptor& kUart1Tx = kArray[3];
};

} // namespace internal

inline constexpr internal::GpioDescriptors kGpioDescriptors{};

} // namespace librmcs::spec::rmcs_board_lite
