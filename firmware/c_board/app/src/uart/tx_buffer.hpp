#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include <main.h>
#include <usart.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/utility/ring_buffer.hpp"

namespace librmcs::firmware::uart {

class TxBuffer {
public:
    static constexpr size_t kBufferSize = 2048;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);
    using IndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kStagingBufferSize = 1024;
    static_assert(kStagingBufferSize <= std::numeric_limits<IndexType>::max());

    static constexpr size_t kMaxIdleCheckpointCount = 256;
    static_assert((kMaxIdleCheckpointCount & (kMaxIdleCheckpointCount - 1)) == 0);

    explicit TxBuffer(
        UART_HandleTypeDef* hal_uart_handle,
        void (*dma_error_callback)(DMA_HandleTypeDef*))
        : hal_uart_handle_(hal_uart_handle)
        , dma_error_callback_(dma_error_callback) {
        core::utility::assert_always(hal_uart_handle_ != nullptr);
        core::utility::assert_always(tx_dma_handle() != nullptr);
    }

    bool try_enqueue(const data::UartDataView& data_view) {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto size = data_view.uart_data.size();
        const auto writable = kBufferSize - static_cast<size_t>(static_cast<IndexType>(in - out));
        if (size > writable)
            return false;

        const auto offset = in & kBufferMask;

        if (data_view.idle_delimited) {
            const auto begin_boundary = in;
            const auto end_boundary = static_cast<IndexType>(in + static_cast<IndexType>(size));

            // Optimization: Reuse the logical idle boundary at the current producer position.
            if (idle_boundary_before_in_) {
                if (size) {
                    // Non-empty: Only append the new 'end'.
                    if (!idle_checkpoints_.push_back(end_boundary))
                        return false;
                }
                // If ZLP (size==0): the existing checkpoint already enforces the idle wait.
            } else {
                if (size) {
                    // Non-empty: Push [begin, end] atomically to ensure isolation on both sides.
                    if (idle_checkpoints_.push_back_n(
                            [&, index = 0]() mutable noexcept {
                                return (index++ == 0) ? begin_boundary : end_boundary;
                            },
                            2, true)
                        != 2) {
                        return false;
                    }
                } else {
                    // ZLP: 'begin' == 'end'. Push single checkpoint to force an IDLE wait.
                    if (!idle_checkpoints_.push_back(begin_boundary))
                        return false;
                }
            }
        }

        if (size) {
            const auto slice = std::min(size, kBufferSize - offset);
            const bool wrapped = size != slice;
            if (wrapped)
                trailing_boundary_segmentable_ = !data_view.idle_delimited;

            std::memcpy(ring_buffer_.data() + offset, data_view.uart_data.data(), slice);
            std::memcpy(ring_buffer_.data(), data_view.uart_data.data() + slice, size - slice);

            in_.store(
                static_cast<IndexType>(in + static_cast<IndexType>(size)),
                std::memory_order::release);

            idle_boundary_before_in_ = data_view.idle_delimited;
        } else {
            // Zero-length non-idle packets should not clear an existing boundary.
            idle_boundary_before_in_ |= data_view.idle_delimited;
        }

        return true;
    }

    bool try_dequeue() {
        if (is_busy_.load(std::memory_order::acquire))
            return false;

        if (!is_idle_)
            is_idle_ =
                timer::timer->check_expired(tx_complete_timepoint_, std::chrono::microseconds(200));

        core::utility::assert_debug_lazy(
            [&]() noexcept { return (tx_dma_handle()->Instance->CR & DMA_SxCR_EN) == 0U; });

        auto out = out_.load(std::memory_order::relaxed);
        if (in_flight_) {
            // For direct ring-buffer DMA, advance out_ only after the previous DMA has finished.
            out = static_cast<IndexType>(out + in_flight_);
            out_.store(out, std::memory_order::release);
            in_flight_ = 0;
        }

        const auto in = in_.load(std::memory_order::acquire);
        const auto readable = static_cast<size_t>(static_cast<IndexType>(in - out));
        if (!readable)
            return false;

        size_t size;
        do {
            size = readable;
            if (auto* idle = idle_checkpoints_.peek_front()) {
                const auto distance = static_cast<size_t>(static_cast<IndexType>(*idle - out));
                core::utility::assert_debug(distance <= readable);
                size = distance;
            }
            size = std::min(size, kStagingBufferSize);

            if (size)
                break;

            // size==0 means out is exactly at a checkpoint boundary.
            // Keep the boundary until the required idle window has elapsed.
            if (!is_idle_)
                return false;

            idle_checkpoints_.pop_front([](const IndexType&) noexcept {});
        } while (true);
        is_idle_ = false;
        is_busy_.store(true, std::memory_order::relaxed);

        const auto offset = out & kBufferMask;
        const auto slice = std::min(size, kBufferSize - offset);
        const bool wrapped = size != slice;

        if (wrapped && !trailing_boundary_segmentable_) {
            // Strict packet must stay contiguous across wrap-around.
            // Flatten into staging and transmit in one DMA shot.
            std::memcpy(staging_buffer_.data(), ring_buffer_.data() + offset, slice);
            std::memcpy(staging_buffer_.data() + slice, ring_buffer_.data(), size - slice);
            out = static_cast<IndexType>(out + static_cast<IndexType>(size));
            out_.store(out, std::memory_order::release);

            core::utility::assert_always(
                HAL_UART_Transmit_DMA(
                    hal_uart_handle_, reinterpret_cast<const uint8_t*>(staging_buffer_.data()),
                    static_cast<uint16_t>(size))
                == HAL_OK);
            bind_tx_dma_error_callback();
            return true;
        }

        // Non-strict path can stream directly from ring; commit progress on completion.
        core::utility::assert_always(
            HAL_UART_Transmit_DMA(
                hal_uart_handle_, reinterpret_cast<const uint8_t*>(ring_buffer_.data() + offset),
                static_cast<uint16_t>(slice))
            == HAL_OK);
        bind_tx_dma_error_callback();
        in_flight_ = static_cast<IndexType>(slice);

        return true;
    }

    void tx_complete_callback() {
        tx_complete_timepoint_ = timer::timer->timepoint();
        is_busy_.store(false, std::memory_order::release);
    }

    void tx_error_callback() {
        core::utility::assert_debug_lazy([]() noexcept { return false; });
        tx_complete_callback();
    }

private:
    DMA_HandleTypeDef* tx_dma_handle() const { return hal_uart_handle_->hdmatx; }

    // HAL_UART_Transmit_DMA resets XferErrorCallback each call; re-bind ours after.
    void bind_tx_dma_error_callback() { tx_dma_handle()->XferErrorCallback = dma_error_callback_; }

    UART_HandleTypeDef* hal_uart_handle_;
    void (*dma_error_callback_)(DMA_HandleTypeDef*);

    alignas(uint32_t) std::array<std::byte, kBufferSize> ring_buffer_{};
    alignas(uint32_t) std::array<std::byte, kStagingBufferSize> staging_buffer_{};

    std::atomic<IndexType> in_{0};
    std::atomic<IndexType> out_{0};
    static_assert(std::atomic<IndexType>::is_always_lock_free);

    IndexType in_flight_{0};
    std::atomic<bool> is_busy_{false};

    // Producer-only state: whether the current in_ position already has a logical idle
    // boundary associated with it.
    bool idle_boundary_before_in_{false};
    bool trailing_boundary_segmentable_{false};
    bool is_idle_{true};
    timer::Timer::TimePoint tx_complete_timepoint_{timer::Timer::TimePoint::min()};

    utility::RingBuffer<IndexType, kMaxIdleCheckpointCount> idle_checkpoints_;
};

} // namespace librmcs::firmware::uart
