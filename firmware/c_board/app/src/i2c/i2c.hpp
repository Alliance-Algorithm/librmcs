#pragma once

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
            write_error(data.slave_address);
            return;
        }

        if (!enqueue_write_request(data)) {
            led::led->downlink_buffer_full();
            write_error(data.slave_address);
        }
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength) {
            write_error(data.slave_address);
            return;
        }

        if (!enqueue_read_request(data)) {
            led::led->downlink_buffer_full();
            write_error(data.slave_address);
        }
    }

    void update() {
        if (transfer_state_.load(std::memory_order::acquire) != TransferState::kIdle) {
            recover_timed_out_transfer();
            if (transfer_state_.load(std::memory_order::acquire) != TransferState::kIdle)
                return;
        }

        Request request{};
        const auto popped =
            request_queue_.pop_front([&request](Request pending) noexcept { request = pending; });
        if (!popped)
            return;

        active_request_ = request;
        transfer_start_timepoint_ = timer::timer->timepoint();
        transfer_state_.store(request.transfer_state(), std::memory_order::release);

        if (!start_dma_transfer(active_request_)) {
            finish_active_request();
            write_error(active_request_.slave_address);
        }
    }

    void tx_complete_callback() {
        const auto state = transfer_state_.load(std::memory_order::acquire);
        core::utility::assert_debug(state == TransferState::kWrite);
        finish_active_request();
    }

    void rx_complete_callback() {
        const auto state = transfer_state_.load(std::memory_order::acquire);
        core::utility::assert_debug(state == TransferState::kRead);
        const auto request = active_request_;
        finish_active_request();
        write_read_result(request);
    }

    void error_callback() {
        const auto state = transfer_state_.load(std::memory_order::acquire);
        if (state == TransferState::kIdle)
            return;

        const auto slave_address = active_request_.slave_address;
        finish_active_request();
        write_error(slave_address);
    }

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;
    static constexpr size_t kRequestQueueSize = 8;
    static constexpr timer::Timer::Duration kTransferTimeout{timer::Timer::kClockFrequency / 10};

    enum class RequestType : uint8_t {
        kWrite,
        kRead,
    };

    enum class TransferState : uint8_t {
        kIdle,
        kWrite,
        kRead,
    };

    struct Request {
        RequestType type = RequestType::kWrite;
        uint8_t slave_address = 0;
        uint16_t data_length = 0;
        bool has_register = false;
        uint8_t reg_address = 0;
        alignas(uint32_t) std::array<std::byte, kMaxDataLength> payload{};

        TransferState transfer_state() const {
            return type == RequestType::kWrite ? TransferState::kWrite : TransferState::kRead;
        }
    };

    bool enqueue_write_request(const data::I2cDataView& data) {
        return request_queue_.emplace_back_n(
                   [&data](std::byte* storage) noexcept {
                       auto& request = *new (storage) Request{};
                       request.type = RequestType::kWrite;
                       request.slave_address = data.slave_address;
                       request.data_length = static_cast<uint16_t>(data.payload.size());
                       request.has_register = data.has_register;
                       request.reg_address = data.reg_address;
                       std::memcpy(
                           request.payload.data(), data.payload.data(), data.payload.size());
                   },
                   1, true)
            == 1;
    }

    bool enqueue_read_request(const data::I2cReadConfigView& data) {
        return request_queue_.emplace_back_n(
                   [&data](std::byte* storage) noexcept {
                       auto& request = *new (storage) Request{};
                       request.type = RequestType::kRead;
                       request.slave_address = data.slave_address;
                       request.data_length = data.read_length;
                       request.has_register = data.has_register;
                       request.reg_address = data.reg_address;
                   },
                   1, true)
            == 1;
    }

    bool start_dma_transfer(Request& request) {
        const auto hal_slave_address = to_hal_slave_address(request.slave_address);

        HAL_StatusTypeDef status;
        if (request.type == RequestType::kWrite) {
            auto* payload = reinterpret_cast<uint8_t*>(request.payload.data());
            status = request.has_register
                       ? HAL_I2C_Mem_Write_DMA(
                             hal_i2c_handle_, hal_slave_address, request.reg_address,
                             I2C_MEMADD_SIZE_8BIT, payload, request.data_length)
                       : HAL_I2C_Master_Transmit_DMA(
                             hal_i2c_handle_, hal_slave_address, payload, request.data_length);
        } else {
            auto* payload = reinterpret_cast<uint8_t*>(read_buffer_.data());
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

        uint8_t slave_address;
        {
            const utility::InterruptLockGuard guard;

            if (transfer_state_.load(std::memory_order::relaxed) == TransferState::kIdle)
                return;

            if (!timer::timer->check_expired(transfer_start_timepoint_, kTransferTimeout))
                return;

            slave_address = active_request_.slave_address;
            abort_dma_if_enabled(hal_i2c_handle_->hdmarx);
            abort_dma_if_enabled(hal_i2c_handle_->hdmatx);
            core::utility::assert_always(HAL_I2C_DeInit(hal_i2c_handle_) == HAL_OK);
            core::utility::assert_always(HAL_I2C_Init(hal_i2c_handle_) == HAL_OK);
            finish_active_request();
        }

        write_error(slave_address);
    }

    static void abort_dma_if_enabled(DMA_HandleTypeDef* hal_dma_handle) {
        core::utility::assert_debug(hal_dma_handle != nullptr);

        if (hal_dma_handle->State == HAL_DMA_STATE_READY)
            return;

        core::utility::assert_always(HAL_DMA_Abort(hal_dma_handle) == HAL_OK);
    }

    void finish_active_request() {
        transfer_start_timepoint_ = timer::Timer::TimePoint::min();
        transfer_state_.store(TransferState::kIdle, std::memory_order::release);
    }

    void write_read_result(const Request& request) {
        auto& serializer = usb::get_serializer();
        const data::I2cDataView result{
            .slave_address = request.slave_address,
            .payload = std::span<const std::byte>{read_buffer_.data(), request.data_length},
            .has_register = request.has_register,
            .reg_address = request.reg_address,
        };
        core::utility::assert_always(
            serializer.write_i2c_read_result(data_id_, result)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    static uint16_t to_hal_slave_address(uint8_t slave_address) {
        return static_cast<uint16_t>(slave_address) << 1;
    }

    void write_error(uint8_t slave_address) {
        auto& serializer = usb::get_serializer();
        core::utility::assert_always(
            serializer.write_i2c_error(data_id_, slave_address)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    const data::DataId data_id_;
    I2C_HandleTypeDef* hal_i2c_handle_;
    utility::RingBuffer<Request, kRequestQueueSize> request_queue_;
    Request active_request_{};
    std::atomic<TransferState> transfer_state_{TransferState::kIdle};
    timer::Timer::TimePoint transfer_start_timepoint_{timer::Timer::TimePoint::min()};
    alignas(uint32_t) std::array<std::byte, kMaxDataLength> read_buffer_{};
};

inline constinit I2c::Lazy i2c0{data::DataId::kI2c0, &hi2c2};

} // namespace librmcs::firmware::i2c
