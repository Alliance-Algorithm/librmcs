#pragma once

#include <algorithm>
#include <array>
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

struct ProcessDataFrame {
    uint32_t epoch;
    uint32_t sequence;
    uint32_t acknowledgement;
    std::span<const std::byte> payload;
};

template <size_t transmit_window>
class ProcessDataChannel {
    static_assert(transmit_window > 0);

public:
    static constexpr size_t kHeaderSize = 16;
    static constexpr size_t kSize = (kHeaderSize + kProtocolBufferSize + 3) & ~size_t{3};
    using Output = std::span<std::byte, kSize>;

    using Frame = ProcessDataFrame;

    static std::optional<Frame> decode(std::span<const std::byte> bytes) {
        if (bytes.size() != kSize || bytes[0] != kVersion || bytes[1] != std::byte{0})
            return std::nullopt;

        const auto length = read(bytes.subspan<2, 2>());
        const auto epoch = read(bytes.subspan<4, 4>());
        const auto sequence = read(bytes.subspan<8, 4>());
        if (length > kProtocolBufferSize || epoch == 0 || (sequence == 0) != (length == 0))
            return std::nullopt;
        return Frame{
            .epoch = epoch,
            .sequence = sequence,
            .acknowledgement = read(bytes.subspan<12, 4>()),
            .payload = bytes.subspan(kHeaderSize, length),
        };
    }

    void reset(uint32_t epoch) {
        epoch_ = epoch;
        transmitted_ = 0;
        received_ = 0;
        pending_head_ = 0;
        pending_count_ = 0;
        next_packet_ = 0;
    }

    uint32_t epoch() const { return epoch_; }
    bool busy() const { return pending_count_ == transmit_window; }

    bool enqueue(std::span<const std::byte> payload) {
        if (epoch_ == 0 || busy() || payload.empty() || payload.size() > kProtocolBufferSize)
            return false;
        transmitted_ = next_sequence(transmitted_);
        auto& packet = pending_at(pending_count_++);
        std::ranges::copy(payload, packet.bytes.begin());
        packet.size = payload.size();
        packet.sequence = transmitted_;
        return true;
    }

    // A valid duplicate or idle frame returns an empty span. Only a new, in-order
    // payload is delivered. A transport ACK does not report peripheral completion.
    std::optional<std::span<const std::byte>> receive(const Frame& frame) {
        if (epoch_ == 0 || frame.epoch != epoch_)
            return std::nullopt;

        const bool fresh = frame.sequence == next_sequence(received_);
        if (!fresh && frame.sequence != 0 && frame.sequence != received_)
            return std::nullopt;

        acknowledge(frame.acknowledgement);
        if (!fresh)
            return std::span<const std::byte>{};

        received_ = frame.sequence;
        return frame.payload;
    }

    void write(Output bytes) {
        std::ranges::fill(bytes, std::byte{0});
        if (epoch_ == 0)
            return;

        bytes[0] = kVersion;
        put(bytes.subspan<4, 4>(), epoch_);
        put(bytes.subspan<12, 4>(), received_);
        if (pending_count_ == 0)
            return;

        // Replay from the oldest unacknowledged packet once the window has been sent.
        if (next_packet_ == pending_count_)
            next_packet_ = 0;
        const auto& packet = pending_at(next_packet_++);
        put(bytes.subspan<2, 2>(), static_cast<uint32_t>(packet.size));
        put(bytes.subspan<8, 4>(), packet.sequence);
        std::ranges::copy(
            std::span{packet.bytes}.first(packet.size), bytes.subspan<kHeaderSize>().begin());
    }

private:
    struct PendingPacket {
        std::array<std::byte, kProtocolBufferSize> bytes{};
        size_t size = 0;
        uint32_t sequence = 0;
    };

    static constexpr std::byte kVersion{1};

    PendingPacket& pending_at(size_t offset) {
        return pending_[(pending_head_ + offset) % transmit_window];
    }

    void acknowledge(uint32_t sequence) {
        for (size_t i = 0; i < pending_count_; ++i) {
            if (pending_at(i).sequence != sequence)
                continue;
            const auto completed = i + 1;
            pending_head_ = (pending_head_ + completed) % transmit_window;
            pending_count_ -= completed;
            next_packet_ = next_packet_ > completed ? next_packet_ - completed : 0;
            return;
        }
    }

    static uint32_t next_sequence(uint32_t value) { return value == UINT32_MAX ? 1 : value + 1; }

    static uint32_t read(std::span<const std::byte> bytes) {
        uint32_t value = 0;
        for (size_t i = 0; i < bytes.size(); ++i)
            value |= std::to_integer<uint32_t>(bytes[i]) << (8U * i);
        return value;
    }

    static void put(std::span<std::byte> bytes, uint32_t value) {
        for (auto& byte : bytes) {
            byte = static_cast<std::byte>(value & 0xffU);
            value >>= 8U;
        }
    }

    std::array<PendingPacket, transmit_window> pending_{};
    size_t pending_head_ = 0;
    size_t pending_count_ = 0;
    size_t next_packet_ = 0;
    uint32_t epoch_ = 0;
    uint32_t transmitted_ = 0;
    uint32_t received_ = 0;
};

using ProcessData = ProcessDataChannel<1>;

} // namespace librmcs::core::protocol::ethercat
