#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <type_traits>

#include <board.h>
#include <hpm_common.h>
#include <hpm_dma_mgr.h>
#include <hpm_dmav2_drv.h>
#include <hpm_l1c_drv.h>
#include <hpm_uart_drv.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/src/utility/ring_buffer.hpp"

namespace librmcs::firmware::uart {

class TxBuffer {
public:
    TxBuffer(UART_Type* uart_base, uint32_t dmamux_src)
        : uart_base_(uart_base) {
        init_dma(dmamux_src);
    }

    static constexpr size_t kBufferSize = 2048;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);

    using BufferIndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<uint16_t>::max());

    static constexpr size_t kMaxIdleCount = 256;

    bool try_enqueue(const data::UartDataView& data_view) {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto writable =
            kBufferSize - static_cast<size_t>(static_cast<BufferIndexType>(in - out));

        const auto size = data_view.uart_data.size();
        if (size > writable)
            return false;

        auto offset = in & kBufferMask;

        if (data_view.idle_delimited) {
            const auto begin_idle = static_cast<BufferIndexType>(offset);
            const auto end_idle = static_cast<BufferIndexType>((offset + size) & kBufferMask);

            // Check if the previous packet ended exactly where this one starts.
            if (auto back = idle_buffer_.peek_back(); back && *back == begin_idle) {
                // Optimization: Deduplicate shared boundary.
                // The existing 'back' serves as 'begin' for this packet.
                if (size) {
                    // Non-empty: Only append the new 'end'.
                    if (!idle_buffer_.push_back(end_idle))
                        return false;
                }
                // If ZLP (size==0): 'begin' == 'end' == 'back'. Existing checkpoint suffices.
            } else {
                if (size) {
                    // Non-empty: Push [begin, end] atomically to ensure isolation on both sides.
                    if (!idle_buffer_.push_back_n(
                            [&, i = 0]() mutable noexcept {
                                return (i++ == 0) ? begin_idle : end_idle;
                            },
                            2, true)) {
                        return false;
                    }
                } else {
                    // ZLP: 'begin' == 'end'. Push single checkpoint to force an IDLE wait.
                    if (!idle_buffer_.push_back(begin_idle))
                        return false;
                }
            }
        }

        auto slice = std::min(size, kBufferSize - offset);
        std::memcpy(data_buffer_.data() + offset, data_view.uart_data.data(), slice);
        std::memcpy(data_buffer_.data(), data_view.uart_data.data() + slice, size - slice);

        in_.store(
            static_cast<BufferIndexType>(in + static_cast<BufferIndexType>(size)),
            std::memory_order::release);

        return true;
    }

    bool try_dequeue() {
        if (dma_channel_is_enable(dma_.base, dma_.channel))
            return false;
        auto out = out_.load(std::memory_order::relaxed);
        if (in_flight_) {
            out = static_cast<BufferIndexType>(out + in_flight_);
            out_.store(out, std::memory_order::release);
            in_flight_ = 0;
        }

        const auto in = in_.load(std::memory_order::acquire);
        const auto readable = static_cast<size_t>(static_cast<BufferIndexType>(in - out));
        if (!readable)
            return false;

        const auto offset = out & kBufferMask;

        size_t size;
        do {
            size = readable;
            if (auto idle = idle_buffer_.peek_front())
                size = static_cast<BufferIndexType>(*idle - offset) & kBufferMask;

            if (size)
                break;

            if (tx_triggered_ && !uart_is_txline_idle(uart_base_))
                return false;

            idle_buffer_.pop_front([](BufferIndexType&&) noexcept {});
        } while (true);
        tx_triggered_ = true;
        uart_clear_txline_idle_flag(uart_base_);

        const auto slice = std::min(size, kBufferSize - offset);
        if (slice == size)
            trigger_dma(data_buffer_.data() + offset, slice, nullptr, 0);
        else
            trigger_dma(data_buffer_.data() + offset, slice, data_buffer_.data(), size - slice);

        in_flight_ = static_cast<BufferIndexType>(size);

        return true;
    }

private:
    void init_dma(uint32_t dmamux_src) {
        dma_mgr_chn_conf_t config;
        dma_mgr_get_default_chn_config(&config);

        config.en_dmamux = true;
        config.dmamux_src = dmamux_src;
        config.priority = DMA_MGR_CHANNEL_PRIORITY_LOW;
        config.src_addr = 0;
        config.dst_addr = reinterpret_cast<uint32_t>(&uart_base_->THR);
        config.src_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.dst_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.src_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_INCREMENT;
        config.dst_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_FIXED;
        config.src_mode = DMA_MGR_HANDSHAKE_MODE_NORMAL;
        config.dst_mode = DMA_MGR_HANDSHAKE_MODE_HANDSHAKE;
        config.src_burst_size = DMA_MGR_NUM_TRANSFER_PER_BURST_1T;

        core::utility::assert_always(
            dma_mgr_request_resource(&dma_) == status_success
            && dma_mgr_setup_channel(&dma_, &config) == status_success
            && dma_mgr_config_linked_descriptor(&dma_, &config, &dma_linked_descriptor_mgr_)
                   == status_success);
        l1c_dc_flush(
            reinterpret_cast<size_t>(&dma_linked_descriptor_), sizeof(dma_linked_descriptor_));
    }

    void trigger_dma(const std::byte* src, size_t size, const std::byte* src2, size_t size2) {
        core::utility::assert_debug(src);
        l1c_dc_flush_cacheline_aligned(src, size);
        auto& ctrl = dma_.base->CHCTRL[dma_.channel];
        ctrl.SRCADDR = reinterpret_cast<uintptr_t>(src);
        ctrl.TRANSIZE = size;

        if (src2) {
            l1c_dc_flush_cacheline_aligned(src2, size2);
            dma_linked_descriptor_.src_addr = reinterpret_cast<uintptr_t>(src2);
            dma_linked_descriptor_.trans_size = size2;
            l1c_dc_flush(
                reinterpret_cast<size_t>(&dma_linked_descriptor_), sizeof(dma_linked_descriptor_));
            ctrl.LLPOINTER = reinterpret_cast<uintptr_t>(&dma_linked_descriptor_);
        } else {
            ctrl.LLPOINTER = 0;
        }

        ctrl.CTRL |= DMAV2_CHCTRL_CTRL_ENABLE_MASK;
    }

    static void l1c_dc_flush_cacheline_aligned(const std::byte* src, uint32_t size) {
        uintptr_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(src);
        uintptr_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(src + size);
        size_t aligned_size = aligned_end - aligned_start;
        l1c_dc_flush(aligned_start, aligned_size);
    }

    alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, kBufferSize> data_buffer_;

    union alignas(HPM_L1C_CACHELINE_SIZE) {
        dma_linked_descriptor_t dma_linked_descriptor_;
        dma_mgr_linked_descriptor_t dma_linked_descriptor_mgr_;
    };
    static_assert(std::is_standard_layout_v<dma_mgr_linked_descriptor_t>);
    static_assert(std::is_standard_layout_v<dma_linked_descriptor_t>);
    static_assert(sizeof(dma_mgr_linked_descriptor_t) == sizeof(dma_linked_descriptor_t));
    static_assert(sizeof(dma_linked_descriptor_t) == HPM_L1C_CACHELINE_SIZE);

    UART_Type* uart_base_;
    dma_resource_t dma_;

    std::atomic<BufferIndexType> in_{0}, out_{0};
    static_assert(std::atomic<BufferIndexType>::is_always_lock_free);
    BufferIndexType in_flight_ = 0;

    bool tx_triggered_ = false;
    utility::RingBuffer<BufferIndexType, kMaxIdleCount> idle_buffer_;
};

} // namespace librmcs::firmware::uart
