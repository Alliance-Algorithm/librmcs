#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <board.h>
#include <hpm_common.h>
#include <hpm_i2c_drv.h>
#include <hpm_soc.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/usb/helper.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::i2c {

class I2c : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<I2c, data::DataId, uintptr_t>;

    explicit I2c(data::DataId data_id, uintptr_t i2c_base)
        : data_id_(data_id)
        , i2c_base_(reinterpret_cast<I2C_Type*>(i2c_base)) {
        board_init_i2c(i2c_base_);
    }

    void handle_downlink_write(const data::I2cDataView& data) {
        if (data.payload.empty())
            return;

        const auto* payload = reinterpret_cast<const uint8_t*>(data.payload.data());
        hpm_stat_t status = status_fail;
        if (data.has_register) {
            uint8_t reg_address = data.reg_address;
            status = i2c_master_address_write(
                i2c_base_, data.slave_address, &reg_address, sizeof(reg_address),
                const_cast<uint8_t*>(payload), static_cast<uint32_t>(data.payload.size()));
        } else {
            status = i2c_master_write(
                i2c_base_, data.slave_address, const_cast<uint8_t*>(payload),
                static_cast<uint32_t>(data.payload.size()));
        }

        if (status != status_success) {
            auto& serializer = usb::get_serializer();
            core::utility::assert_always(
                serializer.write_i2c_error(data_id_, data.slave_address)
                != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            return;
        }
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength)
            return;

        auto* payload = reinterpret_cast<uint8_t*>(read_buffer_.data());
        hpm_stat_t status = status_fail;
        if (data.has_register) {
            uint8_t reg_address = data.reg_address;
            status = i2c_master_address_read(
                i2c_base_, data.slave_address, &reg_address, sizeof(reg_address), payload,
                data.read_length);
        } else {
            status = i2c_master_read(i2c_base_, data.slave_address, payload, data.read_length);
        }

        auto& serializer = usb::get_serializer();
        if (status != status_success) {
            core::utility::assert_always(
                serializer.write_i2c_error(data_id_, data.slave_address)
                != core::protocol::Serializer::SerializeResult::kInvalidArgument);
            return;
        }

        core::utility::assert_always(
            serializer.write_i2c_read_result(
                data_id_,
                {
                    .slave_address = data.slave_address,
                    .payload = std::span<const std::byte>{read_buffer_.data(), data.read_length},
                    .has_register = data.has_register,
                    .reg_address = data.reg_address
        })
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;

    const data::DataId data_id_;
    I2C_Type* i2c_base_;
    std::array<std::byte, kMaxDataLength> read_buffer_{};
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
