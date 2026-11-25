#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "core/src/protocol/constant.hpp"

namespace librmcs::core::protocol::ethercat {

// Private engineering identity. Do not use as a production ETG Vendor ID.
inline constexpr uint32_t kVendorId = 0;
inline constexpr uint32_t kProductCode = 0x554c5452;
inline constexpr uint32_t kRevision = 1;
inline constexpr uint16_t kSoftwareVersionIndex = 0x100a;

class ProcessData {
public:
    static constexpr size_t kHeaderSize = 16;
    static constexpr size_t kSize = (kHeaderSize + kProtocolBufferSize + 3) & ~size_t{3};
    using Output = std::span<std::byte, kSize>;

    struct Frame {
        uint32_t epoch;
        uint32_t sequence;
        uint32_t acknowledgement;
        std::span<const std::byte> payload;
    };

    static std::optional<Frame> decode(std::span<const std::byte> bytes) {
        if (bytes.size() != kSize || bytes[0] != kVersion || bytes[1] != std::byte{0})
            return std::nullopt;

        const auto length = read_little_endian(bytes.subspan<2, 2>());
        const auto epoch = read_little_endian(bytes.subspan<4, 4>());
        const auto sequence = read_little_endian(bytes.subspan<8, 4>());
        if (length > kProtocolBufferSize || epoch == 0 || (sequence == 0) != (length == 0))
            return std::nullopt;
        return Frame{
            .epoch = epoch,
            .sequence = sequence,
            .acknowledgement = read_little_endian(bytes.subspan<12, 4>()),
            .payload = bytes.subspan(kHeaderSize, length),
        };
    }

    static void encode(Output bytes, const Frame& frame) {
        std::ranges::fill(bytes, std::byte{0});
        if (frame.epoch == 0)
            return;

        bytes[0] = kVersion;
        write_little_endian(bytes.subspan<4, 4>(), frame.epoch);
        write_little_endian(bytes.subspan<12, 4>(), frame.acknowledgement);
        if (!frame.payload.empty()) {
            write_little_endian(bytes.subspan<2, 2>(), static_cast<uint32_t>(frame.payload.size()));
            write_little_endian(bytes.subspan<8, 4>(), frame.sequence);
            std::ranges::copy(frame.payload, bytes.subspan<kHeaderSize>().begin());
        }
    }

private:
    static constexpr std::byte kVersion{1};

    static uint32_t read_little_endian(std::span<const std::byte> bytes) {
        uint32_t value = 0;
        for (size_t i = 0; i < bytes.size(); ++i)
            value |= std::to_integer<uint32_t>(bytes[i]) << (8U * i);
        return value;
    }

    static void write_little_endian(std::span<std::byte> bytes, uint32_t value) {
        for (auto& byte : bytes) {
            byte = static_cast<std::byte>(value & 0xffU);
            value >>= 8U;
        }
    }
};

} // namespace librmcs::core::protocol::ethercat
