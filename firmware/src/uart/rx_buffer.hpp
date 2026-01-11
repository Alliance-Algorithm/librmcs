#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <board.h>
#include <hpm_common.h>
#include <hpm_dma_mgr.h>
#include <hpm_dmav2_drv.h>
#include <hpm_l1c_drv.h>
#include <hpm_uart_drv.h>
#include <type_traits>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace librmcs::firmware::uart {

template <typename T>
class RxBuffer {
public:
    RxBuffer(UART_Type* uart_base, uint32_t dmamux_src)
        : uart_base_(uart_base) {
        init_dma(dmamux_src);
    };

    static constexpr size_t kBufferSize = 2048;
    static constexpr size_t kBufferMask = kBufferSize - 1;
    static_assert((kBufferSize & (kBufferSize - 1)) == 0);

    static constexpr size_t kBufferTriggerIrqSize = HPM_L1C_CACHELINE_SIZE;
    static constexpr size_t kDmaTransSize = 2 * kBufferTriggerIrqSize;
    static constexpr size_t kDmaDescriptorCount = kBufferSize / kDmaTransSize;
    static_assert(kBufferSize % kDmaTransSize == 0);

    static constexpr size_t kMinFragmentSize = 32;
    static constexpr size_t kMaxFragmentSize = kMinFragmentSize + kBufferTriggerIrqSize - 1;
    static constexpr size_t kProtocolMaxPayloadSize =
        core::protocol::kProtocolBufferSize - sizeof(core::protocol::UartHeaderExtended);
    static_assert(kMaxFragmentSize <= kProtocolMaxPayloadSize);

    using BufferIndexType = uint16_t;
    static_assert(kBufferSize <= std::numeric_limits<uint16_t>::max());

    void rx_idle_callback() { try_dequeue(true); }

private:
    void dma_tc_half_tc_callback() { try_dequeue(false); }

    void init_dma(uint32_t dmamux_src) {
        dma_mgr_chn_conf_t config;
        dma_mgr_get_default_chn_config(&config);

        config.en_dmamux = true;
        config.dmamux_src = dmamux_src;
        config.priority = DMA_MGR_CHANNEL_PRIORITY_LOW;
        config.src_addr = reinterpret_cast<uintptr_t>(&uart_base_->RBR);
        config.dst_addr = reinterpret_cast<uintptr_t>(data_buffer_.data());
        config.src_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.dst_width = DMA_MGR_TRANSFER_WIDTH_BYTE;
        config.src_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_FIXED;
        config.dst_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_INCREMENT;
        config.src_mode = DMA_MGR_HANDSHAKE_MODE_HANDSHAKE;
        config.dst_mode = DMA_MGR_HANDSHAKE_MODE_NORMAL;
        config.src_burst_size = DMA_MGR_NUM_TRANSFER_PER_BURST_1T;
        config.size_in_byte = kDmaTransSize;
        config.linked_ptr = reinterpret_cast<uintptr_t>(&dma_linked_descriptors[1]);
        static_assert(dma_linked_descriptors.size() >= 2);
        config.interrupt_mask = DMA_INTERRUPT_MASK_ABORT | DMA_INTERRUPT_MASK_ERROR;

        core::utility::assert_always(
            dma_mgr_request_resource(&dma_) == status_success
            && dma_mgr_setup_channel(&dma_, &config) == status_success);

        for (size_t i = 0; i < dma_linked_descriptors.size(); i++) {
            config.linked_ptr = reinterpret_cast<uintptr_t>(
                &dma_linked_descriptors[(i + 1) % dma_linked_descriptors.size()]);
            core::utility::assert_always(
                dma_mgr_config_linked_descriptor(&dma_, &config, &dma_linked_descriptors[i])
                == status_success);
            config.dst_addr += kDmaTransSize;
        }
        l1c_dc_flush(
            reinterpret_cast<size_t>(dma_linked_descriptors.data()),
            sizeof(dma_linked_descriptors));

        auto callback = [](DMA_Type* /*base*/, uint32_t /*channel*/, void* user_data) {
            static_cast<RxBuffer*>(user_data)->dma_tc_half_tc_callback();
        };
        core::utility::assert_always(
            dma_mgr_install_chn_tc_callback(&dma_, callback, this) == status_success
            && dma_mgr_install_chn_half_tc_callback(&dma_, callback, this) == status_success
            && dma_mgr_enable_dma_irq_with_priority(&dma_, 1) == status_success
            && dma_mgr_enable_channel(&dma_) == status_success);
    }

    bool try_dequeue(bool is_idle) {
        const auto in = update_in();
        const auto out = out_.load(std::memory_order::acquire);

        const auto readable = static_cast<size_t>(static_cast<BufferIndexType>(in - out));
        if (!is_idle && readable < kMinFragmentSize)
            return false;

        if (readable > kProtocolMaxPayloadSize) [[unlikely]] {
            // Abnormal condition: ISR latency (or DMA callback starvation) allowed the RX backlog
            // to exceed what a single protocol UART payload can carry.
#ifndef NDEBUG
            // Fail-fast in debug builds to catch timing/interrupt issues early.
            core::utility::assert_failed_always();
#endif
            // Release fallback: discard the accumulated bytes to resync the stream.
            // TODO: add a drop counter / telemetry hook.
        } else {
            const auto offset = out & kBufferMask;
            const auto slice = std::min(readable, kBufferSize - offset);

            if (slice == readable) {
                l1c_dc_invalidate_cacheline_aligned(data_buffer_.data() + offset, slice);
                static_cast<T*>(this)->handle_uplink(
                    {data_buffer_.data() + offset, slice}, {}, is_idle);
            } else {
                l1c_dc_invalidate_cacheline_aligned(data_buffer_.data() + offset, slice);
                l1c_dc_invalidate_cacheline_aligned(data_buffer_.data(), readable - slice);
                static_cast<T*>(this)->handle_uplink(
                    {data_buffer_.data() + offset, slice}, {data_buffer_.data(), readable - slice},
                    is_idle);
            }
        }

        out_.store(
            static_cast<BufferIndexType>(out + static_cast<BufferIndexType>(readable)),
            std::memory_order::release);

        return true;
    }

    BufferIndexType update_in() {
        const auto in = in_.load(std::memory_order::relaxed);

        const size_t current_offset = dma_.base->CHCTRL[dma_.channel].DSTADDR
                                    - reinterpret_cast<uintptr_t>(data_buffer_.data());
        core::utility::assert_debug(current_offset < kBufferSize);

        auto new_in = static_cast<BufferIndexType>((in & ~kBufferMask) | current_offset);
        if (new_in < in)
            new_in += kBufferSize;

        in_.store(new_in, std::memory_order::release);
        return new_in;
    }

    static void l1c_dc_invalidate_cacheline_aligned(const std::byte* src, uint32_t size) {
        uintptr_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(src);
        uintptr_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(src + size);
        size_t aligned_size = aligned_end - aligned_start;
        l1c_dc_invalidate(aligned_start, aligned_size);
    }

    alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, kBufferSize> data_buffer_;

    alignas(HPM_L1C_CACHELINE_SIZE)
        std::array<dma_mgr_linked_descriptor_t, kDmaDescriptorCount> dma_linked_descriptors;
    static_assert(std::is_standard_layout_v<dma_mgr_linked_descriptor_t>);
    static_assert(std::is_standard_layout_v<dma_linked_descriptor_t>);
    static_assert(sizeof(dma_mgr_linked_descriptor_t) == sizeof(dma_linked_descriptor_t));
    static_assert(sizeof(dma_linked_descriptor_t) == HPM_L1C_CACHELINE_SIZE);

    UART_Type* uart_base_;
    dma_resource_t dma_;

    std::atomic<BufferIndexType> in_{0}, out_{0};
    static_assert(std::atomic<BufferIndexType>::is_always_lock_free);
};

} // namespace librmcs::firmware::uart
