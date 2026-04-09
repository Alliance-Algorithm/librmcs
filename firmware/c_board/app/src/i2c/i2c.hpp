#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <i2c.h>
#include <stm32f4xx_hal_def.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

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

        const auto status = write_blocking(data);
        if (status != HAL_OK)
            write_error(data.slave_address);
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength) {
            write_error(data.slave_address);
            return;
        }

        const auto status = read_blocking(data);
        if (status != HAL_OK) {
            write_error(data.slave_address);
            return;
        }

        auto& serializer = usb::get_serializer();
        const data::I2cDataView result{
            .slave_address = data.slave_address,
            .payload = std::span<const std::byte>{read_buffer_.data(), data.read_length},
            .has_register = data.has_register,
            .reg_address = data.reg_address,
        };
        core::utility::assert_always(
            serializer.write_i2c_read_result(data_id_, result)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    void update() {}

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;
    static constexpr uint32_t kTransferTimeoutMs = 100U;

    HAL_StatusTypeDef write_blocking(const data::I2cDataView& data) {
        const auto hal_slave_address = to_hal_slave_address(data.slave_address);
        auto* payload = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.payload.data()));
        const auto payload_length = static_cast<uint16_t>(data.payload.size());

        return data.has_register
                 ? HAL_I2C_Mem_Write(
                       hal_i2c_handle_, hal_slave_address, data.reg_address, I2C_MEMADD_SIZE_8BIT,
                       payload, payload_length, kTransferTimeoutMs)
                 : HAL_I2C_Master_Transmit(
                       hal_i2c_handle_, hal_slave_address, payload, payload_length,
                       kTransferTimeoutMs);
    }

    HAL_StatusTypeDef read_blocking(const data::I2cReadConfigView& data) {
        const auto hal_slave_address = to_hal_slave_address(data.slave_address);
        auto* payload = reinterpret_cast<uint8_t*>(read_buffer_.data());

        return data.has_register
                 ? HAL_I2C_Mem_Read(
                       hal_i2c_handle_, hal_slave_address, data.reg_address, I2C_MEMADD_SIZE_8BIT,
                       payload, data.read_length, kTransferTimeoutMs)
                 : HAL_I2C_Master_Receive(
                       hal_i2c_handle_, hal_slave_address, payload, data.read_length,
                       kTransferTimeoutMs);
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
    std::array<std::byte, kMaxDataLength> read_buffer_{};
};

inline constinit I2c::Lazy i2c0{data::DataId::kI2c0, &hi2c2};

} // namespace librmcs::firmware::i2c
