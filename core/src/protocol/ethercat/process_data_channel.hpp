#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/ethercat/process_data.hpp"

namespace librmcs::core::protocol::ethercat {

template <size_t transmit_window>
class ProcessDataChannel {
    static_assert(transmit_window > 0);

public:
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

    // Valid idle and duplicate frames return an empty span.
    std::optional<std::span<const std::byte>> receive(const ProcessData::Frame& frame) {
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

    void write(ProcessData::Output bytes) {
        ProcessData::Frame frame{
            .epoch = epoch_, .sequence = 0, .acknowledgement = received_, .payload = {}};
        if (pending_count_ != 0) {
            // Replay from the oldest unacknowledged packet after sending the window.
            if (next_packet_ == pending_count_)
                next_packet_ = 0;
            const auto& packet = pending_at(next_packet_++);
            frame.sequence = packet.sequence;
            frame.payload = std::span{packet.bytes}.first(packet.size);
        }
        ProcessData::encode(bytes, frame);
    }

private:
    struct PendingPacket {
        std::array<std::byte, kProtocolBufferSize> bytes{};
        size_t size = 0;
        uint32_t sequence = 0;
    };

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

    std::array<PendingPacket, transmit_window> pending_{};
    size_t pending_head_ = 0;
    size_t pending_count_ = 0;
    size_t next_packet_ = 0;
    uint32_t epoch_ = 0;
    uint32_t transmitted_ = 0;
    uint32_t received_ = 0;
};

} // namespace librmcs::core::protocol::ethercat
