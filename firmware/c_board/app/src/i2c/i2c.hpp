#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <i2c.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_dma.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/interrupt_lock.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"
#include "firmware/c_board/app/src/utility/ring_buffer.hpp"

namespace librmcs::firmware::i2c {

class I2c : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<I2c, data::DataId, I2C_HandleTypeDef*>;

    I2c(data::DataId data_id, I2C_HandleTypeDef* hal_i2c_handle)
        : data_id_(data_id)
        , hal_i2c_handle_(hal_i2c_handle) {
        core::utility::assert_always(hal_i2c_handle_ != nullptr);
        core::utility::assert_always(hal_i2c_handle_->hdmarx != nullptr);
        core::utility::assert_always(hal_i2c_handle_->hdmatx != nullptr);
    }

    void handle_downlink_write(const data::I2cDataView& data) {
        if (data.payload.empty())
            return;

        if (data.payload.size() > kMaxDataLength) {
            publish_error(make_error_view(data));
            return;
        }

        if (!enqueue_write_request(data)) {
            led::led->downlink_buffer_full();
            publish_error(make_error_view(data));
        } else {
            try_start_next_request();
        }
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength) {
            publish_error(make_error_view(data));
            return;
        }

        if (!enqueue_read_request(data)) {
            led::led->downlink_buffer_full();
            publish_error(make_error_view(data));
        } else {
            try_start_next_request();
        }
    }

    void update() {
        try_flush_uplink();

        if (transfer_state_.load(std::memory_order::acquire) != TransferState::kIdle)
            recover_timed_out_transfer();

        try_start_next_request();
    }

    void tx_complete_callback() {
        {
            const utility::InterruptLockGuard guard;
            if (transfer_state_.load(std::memory_order::relaxed) != TransferState::kWrite)
                return;

            finish_active_request_locked();
        }

        try_start_next_request();
    }

    void rx_complete_callback() {
        Request request{};
        {
            const utility::InterruptLockGuard guard;
            if (transfer_state_.load(std::memory_order::relaxed) != TransferState::kRead)
                return;

            request = active_request_;
            std::memcpy(read_result_buffer_.data(), read_dma_buffer_.data(), request.data_length);
            finish_active_request_locked();
        }

        try_start_next_request();
        publish_read_result(
            request, {read_result_buffer_.data(), static_cast<std::size_t>(request.data_length)});
    }

    void error_callback() {
        const auto state = transfer_state_.load(std::memory_order::acquire);
        if (state == TransferState::kIdle)
            return;

        const auto error = make_error_view(active_request_);
        finish_active_request();
        try_start_next_request();
        publish_error(error);
    }

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;
    static constexpr size_t kRequestQueueSize = 32;
    static constexpr size_t kPendingUplinkQueueSize = 32;
    static constexpr size_t kPayloadChunkSize = 64;
    static constexpr size_t kPayloadChunkCount = 32;
    static constexpr size_t kPendingUplinkQueueMask = kPendingUplinkQueueSize - 1;
    static constexpr size_t kPayloadChunkMask = kPayloadChunkCount - 1;
    static constexpr timer::Timer::Duration kTransferTimeout{timer::Timer::kClockFrequency / 10};

    static_assert((kPayloadChunkCount & (kPayloadChunkCount - 1)) == 0);
    static_assert((kPendingUplinkQueueSize & (kPendingUplinkQueueSize - 1)) == 0);

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
        alignas(uint32_t) std::array<std::byte, kMaxDataLength> payload{};
    };

    static data::I2cErrorView make_error_view(const data::I2cDataView& data) {
        return data::I2cErrorView{
            .slave_address = data.slave_address,
            .data_length = static_cast<uint16_t>(data.payload.size()),
            .has_register = data.has_register,
            .reg_address = data.reg_address,
            .is_read = false,
        };
    }

    static data::I2cErrorView make_error_view(const data::I2cReadConfigView& data) {
        return data::I2cErrorView{
            .slave_address = data.slave_address,
            .data_length = data.read_length,
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

    void copy_payload_to_dma_buffer(const Request& request) {
        size_t copied_size = 0;
        size_t remaining_size = request.data_length;

        for (uint8_t chunk = 0; chunk < request.payload_chunk_count; ++chunk) {
            const auto chunk_index = advance_chunk_index(request.payload_first_chunk, chunk);
            const auto chunk_size = std::min(remaining_size, kPayloadChunkSize);
            std::memcpy(
                write_dma_buffer_.data() + copied_size, payload_chunks_[chunk_index].data(),
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

        core::utility::assert_debug(request.payload_first_chunk == payload_chunk_tail_);
        payload_chunk_tail_ = advance_chunk_index(payload_chunk_tail_, request.payload_chunk_count);
        used_payload_chunks_ =
            static_cast<uint8_t>(used_payload_chunks_ - request.payload_chunk_count);
    }

    void try_start_next_request() {
        if (transfer_state_.load(std::memory_order::acquire) != TransferState::kIdle)
            return;

        Request request{};
        {
            const utility::InterruptLockGuard guard;
            if (transfer_state_.load(std::memory_order::relaxed) != TransferState::kIdle)
                return;

            const auto popped = request_queue_.pop_front(
                [&request](Request pending) noexcept { request = pending; });
            if (!popped)
                return;

            active_request_ = request;
            transfer_start_timepoint_ = timer::timer->timepoint();
            transfer_state_.store(request.transfer_state(), std::memory_order::release);
        }

        if (!start_dma_transfer(active_request_)) {
            const auto error = make_error_view(active_request_);
            recover_active_request();
            publish_error(error);
        }
    }

    bool start_dma_transfer(Request& request) {
        const auto hal_slave_address = to_hal_slave_address(request.slave_address);

        HAL_StatusTypeDef status;
        if (request.type == RequestType::kWrite) {
            copy_payload_to_dma_buffer(request);
            auto* payload = reinterpret_cast<uint8_t*>(write_dma_buffer_.data());
            status = request.has_register
                       ? HAL_I2C_Mem_Write_DMA(
                             hal_i2c_handle_, hal_slave_address, request.reg_address,
                             I2C_MEMADD_SIZE_8BIT, payload, request.data_length)
                       : HAL_I2C_Master_Transmit_DMA(
                             hal_i2c_handle_, hal_slave_address, payload, request.data_length);
        } else {
            auto* payload = reinterpret_cast<uint8_t*>(read_dma_buffer_.data());
            status = request.has_register
                       ? HAL_I2C_Mem_Read_DMA(
                             hal_i2c_handle_, hal_slave_address, request.reg_address,
                             I2C_MEMADD_SIZE_8BIT, payload, request.data_length)
                       : HAL_I2C_Master_Receive_DMA(
                             hal_i2c_handle_, hal_slave_address, payload, request.data_length);
        }

        return status == HAL_OK;
    }

    void recover_timed_out_transfer() {
        if (!timer::timer->check_expired(transfer_start_timepoint_, kTransferTimeout))
            return;

        data::I2cErrorView error{};
        {
            const utility::InterruptLockGuard guard;

            if (transfer_state_.load(std::memory_order::relaxed) == TransferState::kIdle)
                return;

            if (!timer::timer->check_expired(transfer_start_timepoint_, kTransferTimeout))
                return;

            error = make_error_view(active_request_);
            recover_active_request_locked();
        }

        publish_error(error);
        try_start_next_request();
    }

    static void abort_dma_if_enabled(DMA_HandleTypeDef* hal_dma_handle) {
        core::utility::assert_debug(hal_dma_handle != nullptr);

        if (hal_dma_handle->State == HAL_DMA_STATE_READY)
            return;

        core::utility::assert_always(HAL_DMA_Abort(hal_dma_handle) == HAL_OK);
    }

    void finish_active_request() {
        const utility::InterruptLockGuard guard;
        finish_active_request_locked();
    }

    void recover_active_request() {
        const utility::InterruptLockGuard guard;
        recover_active_request_locked();
    }

    void recover_active_request_locked() {
        abort_dma_if_enabled(hal_i2c_handle_->hdmarx);
        abort_dma_if_enabled(hal_i2c_handle_->hdmatx);
        core::utility::assert_always(HAL_I2C_DeInit(hal_i2c_handle_) == HAL_OK);
        core::utility::assert_always(HAL_I2C_Init(hal_i2c_handle_) == HAL_OK);
        finish_active_request_locked();
    }

    void finish_active_request_locked() {
        release_payload_chunks(active_request_);
        active_request_ = {};
        transfer_start_timepoint_ = timer::Timer::TimePoint::min();
        transfer_state_.store(TransferState::kIdle, std::memory_order::release);
    }

    bool enqueue_read_result_uplink(const Request& request, std::span<const std::byte> payload) {
        core::utility::assert_debug(payload.size() == request.data_length);

        const utility::InterruptLockGuard guard;
        if (pending_uplink_count_ == kPendingUplinkQueueSize)
            return false;

        auto& entry = pending_uplinks_[pending_uplink_head_];
        entry.type = UplinkType::kReadResult;
        entry.slave_address = request.slave_address;
        entry.data_length = request.data_length;
        entry.has_register = request.has_register;
        entry.reg_address = request.reg_address;
        if (!payload.empty()) {
            std::memcpy(entry.payload.data(), payload.data(), payload.size());
        }

        pending_uplink_head_ = advance_pending_uplink_index(pending_uplink_head_);
        pending_uplink_count_ = static_cast<uint8_t>(pending_uplink_count_ + 1);
        return true;
    }

    bool enqueue_error_uplink(const data::I2cErrorView& error) {
        const utility::InterruptLockGuard guard;
        if (pending_uplink_count_ == kPendingUplinkQueueSize)
            return false;

        auto& entry = pending_uplinks_[pending_uplink_head_];
        entry.type = UplinkType::kError;
        entry.slave_address = error.slave_address;
        entry.data_length = error.data_length;
        entry.has_register = error.has_register;
        entry.reg_address = error.reg_address;
        entry.is_read = error.is_read;

        pending_uplink_head_ = advance_pending_uplink_index(pending_uplink_head_);
        pending_uplink_count_ = static_cast<uint8_t>(pending_uplink_count_ + 1);
        return true;
    }

    bool has_pending_uplink() const {
        const utility::InterruptLockGuard guard;
        return pending_uplink_count_ != 0;
    }

    core::protocol::Serializer::SerializeResult
        serialize_read_result(const Request& request, std::span<const std::byte> payload) {
        auto& serializer = usb::get_serializer();
        const data::I2cDataView result{
            .slave_address = request.slave_address,
            .payload = payload,
            .has_register = request.has_register,
            .reg_address = request.reg_address,
        };
        return serializer.write_i2c_read_result(data_id_, result);
    }

    core::protocol::Serializer::SerializeResult serialize_error(const data::I2cErrorView& error) {
        auto& serializer = usb::get_serializer();
        return serializer.write_i2c_error(data_id_, error);
    }

    bool publish_read_result(const Request& request, std::span<const std::byte> payload) {
        if (has_pending_uplink()) {
            if (enqueue_read_result_uplink(request, payload))
                return true;

            led::led->uplink_buffer_full();
            return false;
        }

        const auto result = serialize_read_result(request, payload);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess)
            return true;
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (enqueue_read_result_uplink(request, payload))
            return true;

        led::led->uplink_buffer_full();
        return false;
    }

    bool publish_error(const data::I2cErrorView& error) {
        if (has_pending_uplink()) {
            if (enqueue_error_uplink(error))
                return true;

            led::led->uplink_buffer_full();
            return false;
        }

        const auto result = serialize_error(error);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess)
            return true;
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (enqueue_error_uplink(error))
            return true;

        led::led->uplink_buffer_full();
        return false;
    }

    void try_flush_uplink() {
        while (true) {
            PendingUplink* pending = nullptr;
            {
                const utility::InterruptLockGuard guard;
                if (pending_uplink_count_ == 0)
                    return;
                pending = &pending_uplinks_[pending_uplink_tail_];
            }

            core::protocol::Serializer::SerializeResult result;
            if (pending->type == UplinkType::kReadResult) {
                const data::I2cDataView payload{
                    .slave_address = pending->slave_address,
                    .payload = {pending->payload.data(), pending->data_length},
                    .has_register = pending->has_register,
                    .reg_address = pending->reg_address,
                };
                result = usb::get_serializer().write_i2c_read_result(data_id_, payload);
            } else {
                result = usb::get_serializer().write_i2c_error(
                    data_id_, data::I2cErrorView{
                                  .slave_address = pending->slave_address,
                                  .data_length = pending->data_length,
                                  .has_register = pending->has_register,
                                  .reg_address = pending->reg_address,
                                  .is_read = pending->is_read,
                              });
            }

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

    static uint16_t to_hal_slave_address(uint8_t slave_address) {
        return static_cast<uint16_t>(slave_address) << 1;
    }

    const data::DataId data_id_;
    I2C_HandleTypeDef* hal_i2c_handle_;
    utility::RingBuffer<Request, kRequestQueueSize> request_queue_;
    Request active_request_{};
    std::atomic<TransferState> transfer_state_{TransferState::kIdle};
    timer::Timer::TimePoint transfer_start_timepoint_{timer::Timer::TimePoint::min()};

    uint8_t payload_chunk_head_{0};
    uint8_t payload_chunk_tail_{0};
    uint8_t used_payload_chunks_{0};
    uint8_t pending_uplink_head_{0};
    uint8_t pending_uplink_tail_{0};
    uint8_t pending_uplink_count_{0};

    alignas(uint32_t)
        std::array<std::array<std::byte, kPayloadChunkSize>, kPayloadChunkCount> payload_chunks_{};
    alignas(uint32_t) std::array<std::byte, kMaxDataLength> write_dma_buffer_{};
    alignas(uint32_t) std::array<std::byte, kMaxDataLength> read_dma_buffer_{};
    alignas(uint32_t) std::array<std::byte, kMaxDataLength> read_result_buffer_{};
    std::array<PendingUplink, kPendingUplinkQueueSize> pending_uplinks_{};
};

// Keep the logical protocol name (`I2C0`) in firmware/host APIs.
// On c_board, that logical channel is implemented by STM32 `I2C2`.
inline constinit I2c::Lazy i2c0{data::DataId::kI2c0, &hi2c2};

} // namespace librmcs::firmware::i2c
