#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <i2c.h>
#include <stm32f4xx_hal_def.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"
#include "firmware/c_board/app/src/utility/ring_buffer.hpp"

namespace librmcs::firmware::i2c {

class I2c : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<I2c, data::DataId, I2C_HandleTypeDef*>;

    I2c(data::DataId data_id, I2C_HandleTypeDef* hal_i2c_handle)
        : data_id_(data_id)
        , hal_i2c_handle_(hal_i2c_handle) {}

    void handle_downlink_write(const data::I2cDataView& data) {
        if (data.payload.empty())
            return;

        if (data.payload.size() > kMaxDataLength) {
            write_error(data.slave_address);
            return;
        }

        if (request_queue_.emplace_back_n(
                [&data](std::byte* storage) noexcept {
                    auto& request = *new (storage) Request{};
                    request.operation = Operation::kWrite;
                    request.slave_address = data.slave_address;
                    request.length = static_cast<uint16_t>(data.payload.size());
                    request.has_register = data.has_register;
                    request.reg_address = data.reg_address;
                    std::memcpy(request.payload.data(), data.payload.data(), data.payload.size());
                },
                1)
            != 0) {
            return;
        }

        led::led->downlink_buffer_full();
        write_error(data.slave_address);
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength) {
            write_error(data.slave_address);
            return;
        }

        if (request_queue_.emplace_back_n(
                [&data](std::byte* storage) noexcept {
                    auto& request = *new (storage) Request{};
                    request.operation = Operation::kRead;
                    request.slave_address = data.slave_address;
                    request.length = data.read_length;
                    request.has_register = data.has_register;
                    request.reg_address = data.reg_address;
                },
                1)
            != 0) {
            return;
        }

        led::led->downlink_buffer_full();
        write_error(data.slave_address);
    }

    void update() {
        while (!transfer_in_progress_.load(std::memory_order::acquire)
               && request_queue_.pop_front([this](const Request& request) noexcept {
                      active_request_ = request;
                      if (!start_active_request())
                          write_error(active_request_.slave_address);
                  })) {}
    }

    void tx_complete_callback() {
        core::utility::assert_debug(transfer_in_progress_.load(std::memory_order::acquire));
        transfer_in_progress_.store(false, std::memory_order::release);
    }

    void rx_complete_callback() {
        core::utility::assert_debug(transfer_in_progress_.load(std::memory_order::acquire));
        core::utility::assert_debug(active_request_.operation == Operation::kRead);

        auto& serializer = usb::get_serializer();
        const data::I2cDataView result{
            .slave_address = active_request_.slave_address,
            .payload = std::span<const std::byte>{read_buffer_.data(), active_request_.length},
            .has_register = active_request_.has_register,
            .reg_address = active_request_.reg_address,
        };
        core::utility::assert_always(
            serializer.write_i2c_read_result(data_id_, result)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);

        transfer_in_progress_.store(false, std::memory_order::release);
    }

    void error_callback() {
        core::utility::assert_debug(transfer_in_progress_.load(std::memory_order::acquire));
        write_error(active_request_.slave_address);
        transfer_in_progress_.store(false, std::memory_order::release);
    }

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;
    static constexpr size_t kRequestQueueSize = 4;

    enum class Operation : uint8_t {
        kWrite,
        kRead,
    };

    struct Request {
        Operation operation = Operation::kWrite;
        uint8_t slave_address = 0;
        uint16_t length = 0;
        bool has_register = false;
        uint8_t reg_address = 0;
        std::array<std::byte, kMaxDataLength> payload{};
    };

    bool start_active_request() {
        const auto hal_slave_address = to_hal_slave_address(active_request_.slave_address);

        transfer_in_progress_.store(true, std::memory_order::release);

        if (active_request_.operation == Operation::kWrite) {
            auto* payload = reinterpret_cast<uint8_t*>(active_request_.payload.data());
            const auto status =
                active_request_.has_register
                    ? HAL_I2C_Mem_Write_IT(
                          hal_i2c_handle_, hal_slave_address, active_request_.reg_address,
                          I2C_MEMADD_SIZE_8BIT, payload, active_request_.length)
                    : HAL_I2C_Master_Transmit_IT(
                          hal_i2c_handle_, hal_slave_address, payload, active_request_.length);
            if (status == HAL_OK)
                return true;
        } else {
            auto* payload = reinterpret_cast<uint8_t*>(read_buffer_.data());
            const auto status =
                active_request_.has_register
                    ? HAL_I2C_Mem_Read_IT(
                          hal_i2c_handle_, hal_slave_address, active_request_.reg_address,
                          I2C_MEMADD_SIZE_8BIT, payload, active_request_.length)
                    : HAL_I2C_Master_Receive_IT(
                          hal_i2c_handle_, hal_slave_address, payload, active_request_.length);
            if (status == HAL_OK)
                return true;
        }

        transfer_in_progress_.store(false, std::memory_order::release);
        return false;
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
    std::atomic<bool> transfer_in_progress_ = false;
    Request active_request_{};
    std::array<std::byte, kMaxDataLength> read_buffer_{};
};

inline constinit I2c::Lazy i2c0{data::DataId::kI2c0, &hi2c2};

} // namespace librmcs::firmware::i2c
