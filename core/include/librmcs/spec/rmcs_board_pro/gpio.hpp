#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include <librmcs/spec/gpio.hpp>

namespace librmcs::spec::rmcs_board_pro {

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
        { 0,     kPwmCapabilities},
        { 1,     kPwmCapabilities},
        { 2,     kPwmCapabilities},
        { 3,     kPwmCapabilities},
        { 4, kDigitalCapabilities},
        { 5, kDigitalCapabilities},
        { 6, kDigitalCapabilities},
        { 7, kDigitalCapabilities},
        { 8, kDigitalCapabilities},
        { 9, kDigitalCapabilities},
        {10, kDigitalCapabilities},
        {11, kDigitalCapabilities},
        {12, kDigitalCapabilities},
        {13, kDigitalCapabilities},
        {14, kDigitalCapabilities},
        {15, kDigitalCapabilities},
        {16, kDigitalCapabilities},
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

    static constexpr const GpioDescriptor& kPwm0 = kArray[0];
    static constexpr const GpioDescriptor& kPwm1 = kArray[1];
    static constexpr const GpioDescriptor& kPwm2 = kArray[2];
    static constexpr const GpioDescriptor& kPwm3 = kArray[3];

    static constexpr const GpioDescriptor& kSpiI2cSocket0 = kArray[4];
    static constexpr const GpioDescriptor& kSpiI2cSocket1 = kArray[5];
    static constexpr const GpioDescriptor& kSpiI2cSocket2 = kArray[6];
    static constexpr const GpioDescriptor& kSpiI2cSocket3 = kArray[7];
    static constexpr const GpioDescriptor& kSpiI2cSocket4 = kArray[8];
    static constexpr const GpioDescriptor& kSpiI2cSocket5 = kArray[9];
    static constexpr const GpioDescriptor& kSpiI2cSocket6 = kArray[10];

    static constexpr const GpioDescriptor& kUart0Rx = kArray[11];
    static constexpr const GpioDescriptor& kUart0Tx = kArray[12];
    static constexpr const GpioDescriptor& kUart1Rx = kArray[13];
    static constexpr const GpioDescriptor& kUart1Tx = kArray[14];
    static constexpr const GpioDescriptor& kUart2Rx = kArray[15];
    static constexpr const GpioDescriptor& kUart2Tx = kArray[16];
};

} // namespace internal

inline constexpr internal::GpioDescriptors kGpioDescriptors{};

} // namespace librmcs::spec::rmcs_board_pro
