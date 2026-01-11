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

namespace librmcs::firmware::usb {

class InterruptSafeBuffer final
    : public core::protocol::ISerializeBuffer
    , private core::utility::Immovable {
public:
    static constexpr size_t kBatchCount = 8;
    static_assert(std::has_single_bit(kBatchCount), "Batch count must be a power of 2");

    constexpr InterruptSafeBuffer() = default;

    std::span<std::byte> allocate(size_t size) noexcept override {
        core::utility::assert_debug(size <= core::protocol::kProtocolBufferSize);

        auto out = out_.load(std::memory_order::relaxed);

        while (true) {
            auto in = in_.load(std::memory_order::relaxed);

            auto readable = in - out;
            if (readable) {
                if (auto result = batches_[(in - 1) & mask].allocate(size))
                    return {result, size};
            }

            auto writeable = kBatchCount - readable - 1;
            if (!writeable) {
                // TODO: buffer full indication hook (LED/log); platform pending.
                return {};
            }

            in_.compare_exchange_weak(in, in + 1, std::memory_order::relaxed);
        }
    }

    static constexpr size_t mask = kBatchCount - 1;

    struct Batch {
        std::byte* allocate(size_t size) {
            size_t written_size_local;

            do {
                written_size_local = written_size.load(std::memory_order::relaxed);
                if (core::protocol::kProtocolBufferSize - written_size_local < size)
                    return nullptr;
            } while (!written_size.compare_exchange_weak(
                written_size_local, written_size_local + size, std::memory_order::relaxed));

            return data + written_size_local;
        }

        std::atomic<size_t> written_size = 0;
        alignas(size_t) std::byte data[core::protocol::kProtocolBufferSize]{};
    };

    Batch* pop_batch() {
        auto in = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (!readable)
            return nullptr;
        auto& batch = batches_[out & mask];
        if (!batch.written_size.load(std::memory_order::relaxed))
            return nullptr;

        std::atomic_signal_fence(std::memory_order_release);
        out_.store(out + 1, std::memory_order::relaxed);

        return &batch;
    }

    void clear() {
        auto in = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (!readable)
            return;

        auto offset = out & mask;
        auto slice = std::min(readable, kBatchCount - offset);

        for (size_t i = 0; i < slice; i++)
            batches_[offset + i].written_size.store(0, std::memory_order::relaxed);
        for (size_t i = 0; i < readable - slice; i++)
            batches_[i].written_size.store(0, std::memory_order::relaxed);

        std::atomic_signal_fence(std::memory_order_release);
        out_.store(in, std::memory_order::relaxed);
    }

private:
    std::atomic<size_t> in_{0}, out_{0};
    static_assert(std::atomic<size_t>::is_always_lock_free);
    Batch batches_[kBatchCount];
};

} // namespace librmcs::firmware::usb
