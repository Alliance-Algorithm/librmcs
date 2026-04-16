#pragma once

#include <cstddef>
#include <cstdint>

#include "core/include/librmcs/data/datas.hpp"
#include "core/include/librmcs/protocol/i2c.hpp"
#include "core/src/utility/bitfield.hpp"

namespace librmcs::core::protocol {

using FieldId = data::DataId;

namespace layouts {

using utility::BitfieldMember;

struct FieldHeaderLayout {
    using Id = utility::BitfieldMember<0, 4, FieldId>;
};

struct FieldHeaderExtendedLayout {
    using IdExtended = utility::BitfieldMember<4, 8, FieldId>;
};

struct CanHeaderLayout {
    using IsFdCan = BitfieldMember<4, 1>; // Currently invalid, reserved only
    using IsExtendedCanId = BitfieldMember<5, 1>;
    using IsRemoteTransmission = BitfieldMember<6, 1>;
    using HasCanData = BitfieldMember<7, 1>;
};

struct CanHeaderStandardLayout {
    using CanId = BitfieldMember<8, 11>;
    using DataLengthCode = BitfieldMember<8 + 13, 3>;
};

struct CanHeaderExtendedLayout {
    using CanId = BitfieldMember<8, 29>;
    using DataLengthCode = BitfieldMember<8 + 29, 3>;
};

struct UartHeaderLayout {
    using IdleDelimited = BitfieldMember<4, 1>;
    using IsExtendedLength = BitfieldMember<5, 1>;
    using DataLength = BitfieldMember<6, 2>;
};

struct UartHeaderExtendedLayout {
    using DataLengthExtended = BitfieldMember<6, 10>;
};

} // namespace layouts

struct FieldHeader
    : utility::Bitfield<1>
    , layouts::FieldHeaderLayout {};

struct FieldHeaderExtended
    : utility::Bitfield<2>
    , layouts::FieldHeaderLayout
    , layouts::FieldHeaderExtendedLayout {};

struct CanHeader
    : utility::Bitfield<1>
    , layouts::CanHeaderLayout {};

struct CanHeaderStandard
    : utility::Bitfield<3>
    , layouts::CanHeaderLayout
    , layouts::CanHeaderStandardLayout {};

struct CanHeaderExtended
    : utility::Bitfield<5>
    , layouts::CanHeaderLayout
    , layouts::CanHeaderExtendedLayout {};

struct UartHeader
    : utility::Bitfield<1>
    , layouts::UartHeaderLayout {};

struct UartHeaderExtended
    : utility::Bitfield<2>
    , layouts::UartHeaderLayout
    , layouts::UartHeaderExtendedLayout {};

struct GpioHeader : utility::Bitfield<2> {
    enum class PayloadEnum : uint8_t {
        kDigitalWriteLow = 0b0000,
        kDigitalWriteHigh = 0b0001,
        kAnalogWrite = 0b0010,
        kDigitalRead = 0b0100,
        kAnalogRead = 0b0110,
        kDigitalReadResultLow = 0b1000,
        kDigitalReadResultHigh = 0b1001,
        kAnalogReadResult = 0b1010,
    };

    using PayloadType = utility::BitfieldMember<4, 4, PayloadEnum>;
    using Channel = utility::BitfieldMember<8, 6>;
    using Pull = utility::BitfieldMember<14, 2, data::GpioPull>;
};

struct GpioReadConfigPayload : utility::Bitfield<2> {
    using Asap = utility::BitfieldMember<0, 1>;
    using RisingEdge = utility::BitfieldMember<1, 1>;
    using FallingEdge = utility::BitfieldMember<2, 1>;
    using PeriodMs = utility::BitfieldMember<3, 13, uint16_t>;
};

struct GpioAnalogPayload : utility::Bitfield<2> {
    using Value = utility::BitfieldMember<0, 16, uint16_t>;
};

struct ImuHeader : utility::Bitfield<1> {
    enum class PayloadEnum : uint8_t {
        kAccelerometer = 0,
        kGyroscope = 1,
    };
    using PayloadType = utility::BitfieldMember<4, 4, PayloadEnum>;
};

struct ImuAccelerometerPayload : utility::Bitfield<6> {
    using X = utility::BitfieldMember<0, 16, int16_t>;
    using Y = utility::BitfieldMember<16, 16, int16_t>;
    using Z = utility::BitfieldMember<32, 16, int16_t>;
};

struct ImuGyroscopePayload : utility::Bitfield<6> {
    using X = utility::BitfieldMember<0, 16, int16_t>;
    using Y = utility::BitfieldMember<16, 16, int16_t>;
    using Z = utility::BitfieldMember<32, 16, int16_t>;
};

struct I2cHeader : utility::Bitfield<3> {
    enum class PayloadEnum : uint8_t {
        kWrite = 0,
        kReadRequest = 1,
        kReadResult = 2,
        kError = 3,
    };

    using PayloadType = utility::BitfieldMember<4, 2, PayloadEnum>;
    using HasRegister = utility::BitfieldMember<6, 1>;
    using ErrorFlag = utility::BitfieldMember<7, 1>;
    using SlaveAddress = utility::BitfieldMember<8, 7>;
    using DataLength = utility::BitfieldMember<15, 9>;

    static constexpr std::size_t kDataLengthBits = DataLength::kBitWidth;
    static constexpr std::uint16_t kMaxDataLength = librmcs::protocol::kI2cMaxDataLength;

    static_assert(kDataLengthBits == librmcs::protocol::kI2cDataLengthBits);
    static_assert(kMaxDataLength == ((1U << kDataLengthBits) - 1U));
};

} // namespace librmcs::core::protocol
