#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <board.h>
#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_csr_drv.h>
#include <hpm_dma_mgr.h>
#include <hpm_i2c.h>
#include <hpm_i2c_drv.h>
#include <hpm_i2c_regs.h>
#include <hpm_l1c_drv.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/usb/helper.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"
#include "firmware/rmcs_board/app/src/utility/ring_buffer.hpp"

namespace librmcs::firmware::i2c {

void i2c_dma_complete_callback(uint32_t channel);

class I2c : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<I2c, data::DataId, uintptr_t>;

    explicit I2c(data::DataId data_id, uintptr_t i2c_base)
        : data_id_(data_id)
        , i2c_base_(reinterpret_cast<I2C_Type*>(i2c_base))
        , i2c_context_(make_i2c_context(i2c_base_))
        , initialized_(board_init_i2c(i2c_base_))
        , dma_ready_(install_dma_callback_for_context(i2c_context_)) {}

    void handle_downlink_write(const data::I2cDataView& data) {
        if (data.payload.empty())
            return;

        if (data.payload.size() > kMaxDataLength) {
            (void)publish_error(make_error_view(data));
            return;
        }

        if (!enqueue_write_request(data))
            (void)publish_error(make_error_view(data));
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength) {
            (void)publish_error(make_error_view(data));
            return;
        }

        if (!enqueue_read_request(data))
            (void)publish_error(make_error_view(data));
    }

    void update() {
        try_flush_uplink();
        (void)try_flush_blocked_uplink();
        (void)try_flush_retained_uplink();
        poll_active_transfer();
        try_flush_uplink();
        (void)try_flush_blocked_uplink();
        (void)try_flush_retained_uplink();

        if (transfer_state_ != TransferState::kIdle || has_buffered_uplink())
            return;

        start_next_request();
    }

    void dma_complete_callback() { dma_complete_.store(true, std::memory_order::release); }

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;
    static constexpr size_t kRequestQueueSize = 32;
    static constexpr size_t kBlockedUplinkQueueSize = 4;
    static constexpr size_t kPendingUplinkQueueSize = 32;
    static constexpr size_t kPayloadChunkSize = 64;
    static constexpr size_t kPayloadChunkCount = 32;
    static constexpr size_t kBlockedUplinkQueueMask = kBlockedUplinkQueueSize - 1;
    static constexpr size_t kPendingUplinkQueueMask = kPendingUplinkQueueSize - 1;
    static constexpr size_t kPayloadChunkMask = kPayloadChunkCount - 1;
    static constexpr uint32_t kTransferTimeoutUs = 100'000U;

    static_assert((kBlockedUplinkQueueSize & (kBlockedUplinkQueueSize - 1)) == 0);
    static_assert((kPendingUplinkQueueSize & (kPendingUplinkQueueSize - 1)) == 0);
    static_assert((kPayloadChunkCount & (kPayloadChunkCount - 1)) == 0);

    enum class RequestType : uint8_t {
        kWrite,
        kRead,
    };

    enum class TransferState : uint8_t {
        kIdle,
        kWrite,
        kRead,
    };

    enum class UplinkType : uint8_t {
        kReadResult,
        kError,
    };

    struct Request {
        RequestType type = RequestType::kWrite;
        uint8_t slave_address = 0;
        uint16_t data_length = 0;
        bool has_register = false;
        uint8_t reg_address = 0;
        uint8_t payload_first_chunk = 0;
        uint8_t payload_chunk_count = 0;

        TransferState transfer_state() const {
            return type == RequestType::kWrite ? TransferState::kWrite : TransferState::kRead;
        }
    };

    struct PendingUplink {
        UplinkType type = UplinkType::kError;
        uint8_t slave_address = 0;
        uint16_t data_length = 0;
        bool has_register = false;
        uint8_t reg_address = 0;
        bool is_read = false;
        std::array<std::byte, kMaxDataLength> payload{};
    };

    static uint16_t clamp_data_length(size_t data_length) {
        return static_cast<uint16_t>(std::min(data_length, static_cast<size_t>(kMaxDataLength)));
    }

    static data::I2cErrorView make_error_view(const data::I2cDataView& data) {
        return data::I2cErrorView{
            .slave_address = data.slave_address,
            .data_length = clamp_data_length(data.payload.size()),
            .has_register = data.has_register,
            .reg_address = data.reg_address,
            .is_read = false,
        };
    }

    static data::I2cErrorView make_error_view(const data::I2cReadConfigView& data) {
        return data::I2cErrorView{
            .slave_address = data.slave_address,
            .data_length = clamp_data_length(data.read_length),
            .has_register = data.has_register,
            .reg_address = data.reg_address,
            .is_read = true,
        };
    }

    static data::I2cErrorView make_error_view(const Request& request) {
        return data::I2cErrorView{
            .slave_address = request.slave_address,
            .data_length = request.data_length,
            .has_register = request.has_register,
            .reg_address = request.reg_address,
            .is_read = request.type == RequestType::kRead,
        };
    }

    static uint8_t required_payload_chunks(size_t payload_size) {
        return static_cast<uint8_t>((payload_size + kPayloadChunkSize - 1) / kPayloadChunkSize);
    }

    static uint8_t advance_chunk_index(uint8_t index, uint8_t chunk_count) {
        return static_cast<uint8_t>((index + chunk_count) & kPayloadChunkMask);
    }

    static uint8_t advance_pending_uplink_index(uint8_t index) {
        return static_cast<uint8_t>((index + 1) & kPendingUplinkQueueMask);
    }

    static uint8_t advance_blocked_uplink_index(uint8_t index) {
        return static_cast<uint8_t>((index + 1) & kBlockedUplinkQueueMask);
    }

    static hpm_i2c_context_t make_i2c_context(I2C_Type* i2c_base) {
        hpm_i2c_context_t context{};
        hpm_i2c_get_default_init_context(&context);
        context.base = i2c_base;
        return context;
    }

    static bool install_dma_callback_for_context(hpm_i2c_context_t& i2c_context) {
        if (hpm_i2c_dma_mgr_install_callback(&i2c_context, i2c_dma_complete_callback)
            != status_success) {
            return false;
        }

        const auto* resource = hpm_i2c_get_dma_mgr_resource(&i2c_context);
        return resource != nullptr && resource->base != nullptr;
    }

    static void cache_writeback_buffer(const std::byte* buffer, uint32_t size) {
        if (!l1c_dc_is_enabled() || size == 0)
            return;

        const uintptr_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(buffer);
        const uintptr_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(buffer + size);
        const auto aligned_size = static_cast<uint32_t>(aligned_end - aligned_start);
        l1c_dc_writeback(static_cast<uint32_t>(aligned_start), aligned_size);
    }

    static void cache_invalidate_buffer(const std::byte* buffer, uint32_t size) {
        if (!l1c_dc_is_enabled() || size == 0)
            return;

        const uintptr_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(buffer);
        const uintptr_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(buffer + size);
        const auto aligned_size = static_cast<uint32_t>(aligned_end - aligned_start);
        l1c_dc_invalidate(static_cast<uint32_t>(aligned_start), aligned_size);
    }

    size_t writable_payload_chunks() const {
        return kPayloadChunkCount - static_cast<size_t>(used_payload_chunks_);
    }

    bool enqueue_write_request(const data::I2cDataView& data) {
        const auto required_chunks = required_payload_chunks(data.payload.size());

        const utility::InterruptLockGuard guard;
        if (request_queue_.writable() == 0 || required_chunks > writable_payload_chunks())
            return false;

        Request request{};
        request.type = RequestType::kWrite;
        request.slave_address = data.slave_address;
        request.data_length = static_cast<uint16_t>(data.payload.size());
        request.has_register = data.has_register;
        request.reg_address = data.reg_address;
        request.payload_first_chunk = payload_chunk_head_;
        request.payload_chunk_count = required_chunks;

        copy_payload_to_pool(request, data.payload);
        reserve_payload_chunks(required_chunks);

        core::utility::assert_always(request_queue_.emplace_back(request));
        return true;
    }

    bool enqueue_read_request(const data::I2cReadConfigView& data) {
        const utility::InterruptLockGuard guard;
        if (request_queue_.writable() == 0)
            return false;

        Request request{};
        request.type = RequestType::kRead;
        request.slave_address = data.slave_address;
        request.data_length = data.read_length;
        request.has_register = data.has_register;
        request.reg_address = data.reg_address;

        core::utility::assert_always(request_queue_.emplace_back(request));
        return true;
    }

    void copy_payload_to_pool(const Request& request, std::span<const std::byte> payload) {
        size_t copied_size = 0;
        size_t remaining_size = payload.size();

        for (uint8_t chunk = 0; chunk < request.payload_chunk_count; ++chunk) {
            const auto chunk_index = advance_chunk_index(request.payload_first_chunk, chunk);
            const auto chunk_size = std::min(remaining_size, kPayloadChunkSize);
            std::memcpy(
                payload_chunks_[chunk_index].data(), payload.data() + copied_size, chunk_size);
            copied_size += chunk_size;
            remaining_size -= chunk_size;
        }
    }

    void copy_payload_to_write_buffer(const Request& request) {
        size_t copied_size = 0;
        size_t remaining_size = request.data_length;

        for (uint8_t chunk = 0; chunk < request.payload_chunk_count; ++chunk) {
            const auto chunk_index = advance_chunk_index(request.payload_first_chunk, chunk);
            const auto chunk_size = std::min(remaining_size, kPayloadChunkSize);
            std::memcpy(
                write_buffer_.data() + copied_size, payload_chunks_[chunk_index].data(),
                chunk_size);
            copied_size += chunk_size;
            remaining_size -= chunk_size;
        }
    }

    void reserve_payload_chunks(uint8_t chunk_count) {
        payload_chunk_head_ = advance_chunk_index(payload_chunk_head_, chunk_count);
        used_payload_chunks_ = static_cast<uint8_t>(used_payload_chunks_ + chunk_count);
    }

    void release_payload_chunks(const Request& request) {
        if (request.type != RequestType::kWrite || request.payload_chunk_count == 0)
            return;

        const utility::InterruptLockGuard guard;
        core::utility::assert_debug(request.payload_first_chunk == payload_chunk_tail_);
        payload_chunk_tail_ = advance_chunk_index(payload_chunk_tail_, request.payload_chunk_count);
        used_payload_chunks_ =
            static_cast<uint8_t>(used_payload_chunks_ - request.payload_chunk_count);
    }

    bool install_dma_callback() {
        if (dma_ready_)
            return true;

        dma_ready_ = install_dma_callback_for_context(i2c_context_);
        return dma_ready_;
    }

    bool try_initialize() {
        if (initialized_)
            return true;

        initialized_ = board_init_i2c(i2c_base_);
        return initialized_;
    }

    static bool should_reinitialize(hpm_stat_t status) {
        switch (status) {
        case status_fail:
        case status_timeout:
        case status_i2c_transmit_not_completed: return true;
        default: return false;
        }
    }

    bool has_blocked_uplink() const {
        const utility::InterruptLockGuard guard;
        return blocked_uplink_count_ != 0;
    }

    bool has_retained_uplink() const {
        const utility::InterruptLockGuard guard;
        return retained_uplink_pending_;
    }

    bool has_buffered_uplink() const {
        const utility::InterruptLockGuard guard;
        return pending_uplink_count_ != 0 || blocked_uplink_count_ != 0 || retained_uplink_pending_;
    }

    static PendingUplink make_read_result_uplink(
        uint8_t slave_address, std::span<const std::byte> payload, bool has_register,
        uint8_t reg_address) {
        PendingUplink uplink{};
        uplink.type = UplinkType::kReadResult;
        uplink.slave_address = slave_address;
        uplink.data_length = static_cast<uint16_t>(payload.size());
        uplink.has_register = has_register;
        uplink.reg_address = reg_address;
        if (!payload.empty()) {
            std::memcpy(uplink.payload.data(), payload.data(), payload.size());
        }
        return uplink;
    }

    static PendingUplink make_error_uplink(const data::I2cErrorView& error) {
        PendingUplink uplink{};
        uplink.type = UplinkType::kError;
        uplink.slave_address = error.slave_address;
        uplink.data_length = error.data_length;
        uplink.has_register = error.has_register;
        uplink.reg_address = error.reg_address;
        uplink.is_read = error.is_read;
        return uplink;
    }

    bool stage_blocked_uplink(const PendingUplink& uplink) {
        const utility::InterruptLockGuard guard;
        if (blocked_uplink_count_ == kBlockedUplinkQueueSize)
            return false;

        blocked_uplinks_[blocked_uplink_head_] = uplink;
        blocked_uplink_head_ = advance_blocked_uplink_index(blocked_uplink_head_);
        blocked_uplink_count_ = static_cast<uint8_t>(blocked_uplink_count_ + 1);
        return true;
    }

    bool retain_uplink(const PendingUplink& uplink, bool overwrite) {
        const utility::InterruptLockGuard guard;
        if (retained_uplink_pending_ && !overwrite)
            return false;

        retained_uplink_ = uplink;
        retained_uplink_pending_ = true;
        return true;
    }

    core::protocol::Serializer::SerializeResult try_publish_uplink(const PendingUplink& pending) {
        if (pending.type == UplinkType::kReadResult) {
            const data::I2cDataView payload{
                .slave_address = pending.slave_address,
                .payload = {pending.payload.data(), pending.data_length},
                .has_register = pending.has_register,
                .reg_address = pending.reg_address,
            };
            return usb::get_serializer().write_i2c_read_result(data_id_, payload);
        }

        return usb::get_serializer().write_i2c_error(
            data_id_, data::I2cErrorView{
                          .slave_address = pending.slave_address,
                          .data_length = pending.data_length,
                          .has_register = pending.has_register,
                          .reg_address = pending.reg_address,
                          .is_read = pending.is_read,
                      });
    }

    bool try_flush_blocked_uplink() {
        while (true) {
            const PendingUplink* blocked = nullptr;
            {
                const utility::InterruptLockGuard guard;
                if (blocked_uplink_count_ == 0)
                    return true;
                blocked = &blocked_uplinks_[blocked_uplink_tail_];
            }

            const auto result = try_publish_uplink(*blocked);
            if (result == core::protocol::Serializer::SerializeResult::kBadAlloc)
                return false;
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);

            const utility::InterruptLockGuard guard;
            blocked_uplink_tail_ = advance_blocked_uplink_index(blocked_uplink_tail_);
            blocked_uplink_count_ = static_cast<uint8_t>(blocked_uplink_count_ - 1);
        }
        return true;
    }

    bool try_flush_retained_uplink() {
        PendingUplink retained{};
        {
            const utility::InterruptLockGuard guard;
            if (!retained_uplink_pending_)
                return true;
            retained = retained_uplink_;
        }

        const auto result = try_publish_uplink(retained);
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc)
            return false;
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);

        const utility::InterruptLockGuard guard;
        retained_uplink_pending_ = false;
        retained_uplink_ = {};
        return true;
    }

    void start_next_request() {
        Request request{};
        {
            const utility::InterruptLockGuard guard;
            const auto popped = request_queue_.pop_front(
                [&request](Request pending) noexcept { request = pending; });
            if (!popped)
                return;
        }

        if (!dma_ready_ && !install_dma_callback()) {
            release_payload_chunks(request);
            (void)publish_error(make_error_view(request), true);
            return;
        }

        if (!try_initialize()) {
            release_payload_chunks(request);
            (void)publish_error(make_error_view(request), true);
            return;
        }

        dma_complete_.store(false, std::memory_order::release);
        const hpm_stat_t status = start_nonblocking_transfer(request);
        if (status != status_success) {
            release_payload_chunks(request);
            if (should_reinitialize(status)) {
                abort_active_dma();
                initialized_ = false;
            }
            (void)publish_error(make_error_view(request), true);
            return;
        }

        if (request.type == RequestType::kWrite)
            release_payload_chunks(request);

        active_request_ = request;
        transfer_state_ = request.transfer_state();
        transfer_start_cycle_ = hpm_csr_get_core_cycle();
    }

    hpm_stat_t start_nonblocking_transfer(const Request& request) {
        if (request.type == RequestType::kWrite) {
            copy_payload_to_write_buffer(request);
            cache_writeback_buffer(write_buffer_.data(), request.data_length);

            auto* payload = reinterpret_cast<uint8_t*>(write_buffer_.data());
            if (request.has_register) {
                return hpm_i2c_master_addr_write_nonblocking(
                    &i2c_context_, request.slave_address, request.reg_address,
                    sizeof(request.reg_address), payload, request.data_length);
            }
            return hpm_i2c_master_write_nonblocking(
                &i2c_context_, request.slave_address, payload, request.data_length);
        }

        auto* payload = reinterpret_cast<uint8_t*>(read_buffer_.data());
        if (request.has_register) {
            return hpm_i2c_master_addr_read_nonblocking(
                &i2c_context_, request.slave_address, request.reg_address,
                sizeof(request.reg_address), payload, request.data_length);
        }
        return hpm_i2c_master_read_nonblocking(
            &i2c_context_, request.slave_address, payload, request.data_length);
    }

    bool transfer_timed_out() const {
        if (transfer_start_cycle_ == 0)
            return false;

        const uint64_t timeout_cycles =
            static_cast<uint64_t>(clock_get_core_clock_ticks_per_us()) * kTransferTimeoutUs;
        return (hpm_csr_get_core_cycle() - transfer_start_cycle_) >= timeout_cycles;
    }

    void abort_active_dma() {
        if (!dma_ready_)
            return;

        auto* resource = hpm_i2c_get_dma_mgr_resource(&i2c_context_);
        if (resource != nullptr && resource->base != nullptr)
            (void)dma_mgr_abort_chn_transfer(resource);

        i2c_dma_disable(i2c_base_);
        i2c_clear_status(i2c_base_, I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK);
        i2c_clear_fifo(i2c_base_);
    }

    void finish_active_transfer() {
        active_request_ = {};
        transfer_state_ = TransferState::kIdle;
        transfer_start_cycle_ = 0;
        dma_complete_.store(false, std::memory_order::release);
    }

    void poll_active_transfer() {
        if (transfer_state_ == TransferState::kIdle)
            return;

        if (transfer_timed_out()) {
            const auto error = make_error_view(active_request_);
            abort_active_dma();
            initialized_ = false;
            finish_active_transfer();
            (void)publish_error(error, true);
            return;
        }

        if (!dma_complete_.load(std::memory_order::acquire))
            return;
        if ((i2c_get_status(i2c_base_) & I2C_STATUS_CMPL_MASK) == 0)
            return;
        if (i2c_get_status(i2c_base_) & I2C_STATUS_BUSBUSY_MASK)
            return;
        i2c_clear_status(i2c_base_, I2C_STATUS_CMPL_MASK);

        const Request request = active_request_;
        finish_active_transfer();

        if (request.data_length != 0 && i2c_get_data_count(i2c_base_) != 0) {
            initialized_ = false;
            (void)publish_error(make_error_view(request), true);
            return;
        }

        if (request.type == RequestType::kRead) {
            cache_invalidate_buffer(read_buffer_.data(), request.data_length);
            (void)publish_read_result(
                request.slave_address, {read_buffer_.data(), request.data_length},
                request.has_register, request.reg_address);
        }
    }

    bool enqueue_uplink(const PendingUplink& uplink) {
        const utility::InterruptLockGuard guard;
        if (pending_uplink_count_ == kPendingUplinkQueueSize)
            return false;

        auto& entry = pending_uplinks_[pending_uplink_head_];
        entry = uplink;

        pending_uplink_head_ = advance_pending_uplink_index(pending_uplink_head_);
        pending_uplink_count_ = static_cast<uint8_t>(pending_uplink_count_ + 1);
        return true;
    }

    bool has_pending_uplink() const {
        const utility::InterruptLockGuard guard;
        return pending_uplink_count_ != 0;
    }

    bool publish_uplink(const PendingUplink& uplink, bool overwrite_retained) {
        if (has_retained_uplink()) {
            if (!try_flush_retained_uplink())
                return retain_uplink(uplink, overwrite_retained);
        }

        if (has_blocked_uplink()) {
            if (stage_blocked_uplink(uplink))
                return true;
            return retain_uplink(uplink, overwrite_retained);
        }

        if (has_pending_uplink()) {
            if (enqueue_uplink(uplink))
                return true;
            if (stage_blocked_uplink(uplink))
                return true;
            return retain_uplink(uplink, overwrite_retained);
        }

        const auto result = try_publish_uplink(uplink);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess)
            return true;

        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (enqueue_uplink(uplink))
            return true;
        if (stage_blocked_uplink(uplink))
            return true;
        return retain_uplink(uplink, overwrite_retained);
    }

    bool publish_read_result(
        uint8_t slave_address, std::span<const std::byte> payload, bool has_register,
        uint8_t reg_address) {
        return publish_uplink(
            make_read_result_uplink(slave_address, payload, has_register, reg_address), true);
    }

    bool publish_error(const data::I2cErrorView& error, bool overwrite_retained = false) {
        return publish_uplink(make_error_uplink(error), overwrite_retained);
    }

    void try_flush_uplink() {
        while (true) {
            const PendingUplink* pending = nullptr;
            {
                const utility::InterruptLockGuard guard;
                if (pending_uplink_count_ == 0)
                    return;
                pending = &pending_uplinks_[pending_uplink_tail_];
            }

            const auto result = try_publish_uplink(*pending);
            if (result == core::protocol::Serializer::SerializeResult::kBadAlloc)
                return;
            core::utility::assert_always(
                result != core::protocol::Serializer::SerializeResult::kInvalidArgument);

            const utility::InterruptLockGuard guard;
            core::utility::assert_debug(pending_uplink_count_ > 0);
            pending_uplink_tail_ = advance_pending_uplink_index(pending_uplink_tail_);
            pending_uplink_count_ = static_cast<uint8_t>(pending_uplink_count_ - 1);
        }
    }

    const data::DataId data_id_;
    I2C_Type* i2c_base_;
    hpm_i2c_context_t i2c_context_{};

    bool initialized_{false};
    bool dma_ready_{false};
    TransferState transfer_state_{TransferState::kIdle};
    uint64_t transfer_start_cycle_{0};
    Request active_request_{};
    std::atomic<bool> dma_complete_{false};

    utility::RingBuffer<Request, kRequestQueueSize> request_queue_;
    alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, kMaxDataLength> write_buffer_{};
    alignas(HPM_L1C_CACHELINE_SIZE) std::array<std::byte, kMaxDataLength> read_buffer_{};
    uint8_t payload_chunk_head_{0};
    uint8_t payload_chunk_tail_{0};
    uint8_t used_payload_chunks_{0};

    uint8_t blocked_uplink_head_{0};
    uint8_t blocked_uplink_tail_{0};
    uint8_t blocked_uplink_count_{0};
    bool retained_uplink_pending_{false};
    uint8_t pending_uplink_head_{0};
    uint8_t pending_uplink_tail_{0};
    uint8_t pending_uplink_count_{0};
    std::array<std::array<std::byte, kPayloadChunkSize>, kPayloadChunkCount> payload_chunks_{};
    std::array<PendingUplink, kBlockedUplinkQueueSize> blocked_uplinks_{};
    PendingUplink retained_uplink_{};
    std::array<PendingUplink, kPendingUplinkQueueSize> pending_uplinks_{};
};

// Convert I2C pointer to base address
// Some boards define BOARD_APP_I2C_BASE as pointer (HPM_I2Cx), others don't define it
#ifndef BOARD_APP_I2C_BASE
inline constinit I2c::Lazy i2c0{data::DataId::kI2c0, HPM_I2C0_BASE};
#else
inline constinit I2c::Lazy i2c0{
    data::DataId::kI2c0, reinterpret_cast<uintptr_t>(BOARD_APP_I2C_BASE)};
#endif

} // namespace librmcs::firmware::i2c
