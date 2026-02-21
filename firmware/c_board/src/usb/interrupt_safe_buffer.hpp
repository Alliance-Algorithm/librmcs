#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <span>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/src/led/led.hpp"

namespace librmcs::firmware::usb {

class InterruptSafeBuffer final
    : public core::protocol::SerializeBuffer
    , private core::utility::Immovable {
public:
    static constexpr size_t kBatchCount = 8;
    static_assert(std::has_single_bit(kBatchCount), "Batch count must be a power of 2");

    static constexpr size_t kMask = kBatchCount - 1;

    constexpr InterruptSafeBuffer() = default;

    std::span<std::byte> allocate(size_t size) noexcept override {
        core::utility::assert_debug(size <= core::protocol::kProtocolBufferSize);
        if (is_locked_.test(std::memory_order::relaxed))
            return {};

        auto out = out_.load(std::memory_order::relaxed);

        while (true) {
            auto in = in_.load(std::memory_order::relaxed);

            auto readable = in - out;
            if (readable) {
                if (auto* result = batches_[(in - 1) & kMask].allocate(size))
                    return {result, size};
            }

            auto writeable = kBatchCount - readable - 1;
            if (!writeable) {
                led::led->uplink_buffer_full();
                return {};
            }

            in_.compare_exchange_weak(in, in + 1, std::memory_order::relaxed);
        }
    }

    class Batch {
    public:
        bool empty() const { return written_size_.load(std::memory_order::relaxed) == 0; }

        std::span<const std::byte> data() const {
            return {data_, written_size_.load(std::memory_order::relaxed)};
        }

        std::byte* allocate(size_t size) {
            size_t written_size_local;

            do {
                written_size_local = written_size_.load(std::memory_order::relaxed);
                if (core::protocol::kProtocolBufferSize - written_size_local < size)
                    return nullptr;
            } while (!written_size_.compare_exchange_weak(
                written_size_local, written_size_local + size, std::memory_order::relaxed));

            return data_ + written_size_local;
        }

        void reset() { written_size_.store(0, std::memory_order::relaxed); }

    private:
        std::atomic<size_t> written_size_ = 0;
        alignas(size_t) std::byte data_[core::protocol::kProtocolBufferSize]{};
    };

    const Batch* pop_batch() {
        auto in = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (!readable)
            return nullptr;
        auto& batch = batches_[out & kMask];
        if (batch.empty())
            return nullptr;

        std::atomic_signal_fence(std::memory_order::release);
        out_.store(out + 1, std::memory_order::relaxed);

        return &batch;
    }

    static void release_batch(const Batch* batch) {
        const_cast<Batch*>(batch)->reset(); // NOLINT(cppcoreguidelines-pro-type-const-cast):
                                            // Compromises made to maintain encapsulation.
    }

    void clear() {
        auto in = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (!readable)
            return;

        auto offset = out & kMask;
        auto slice = std::min(readable, kBatchCount - offset);

        for (size_t i = 0; i < slice; i++)
            batches_[offset + i].reset();
        for (size_t i = 0; i < readable - slice; i++)
            batches_[i].reset();

        std::atomic_signal_fence(std::memory_order::release);
        out_.store(in, std::memory_order::relaxed);
    }

    bool try_lock() { return !is_locked_.test_and_set(std::memory_order::relaxed); }

    bool try_unlock_and_clear() {
        if (!is_locked_.test(std::memory_order::relaxed))
            return false;

        // Unlocking drops stale queued batches from the last not-ready cycle before
        // new ISR writes are accepted.
        clear();
        is_locked_.clear(std::memory_order::relaxed);
        return true;
    }

private:
    std::atomic_flag is_locked_;
    std::atomic<size_t> in_{0}, out_{0};
    static_assert(std::atomic<size_t>::is_always_lock_free);
    Batch batches_[kBatchCount];
};

} // namespace librmcs::firmware::usb
