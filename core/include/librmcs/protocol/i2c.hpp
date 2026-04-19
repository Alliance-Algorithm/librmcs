#pragma once

#include <cstddef>
#include <cstdint>

namespace librmcs::protocol {

inline constexpr std::size_t kI2cDataLengthBits = 9;
inline constexpr std::uint16_t kI2cMaxDataLength =
    static_cast<std::uint16_t>((1U << kI2cDataLengthBits) - 1U);

} // namespace librmcs::protocol
