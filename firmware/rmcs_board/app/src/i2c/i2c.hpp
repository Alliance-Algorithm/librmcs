#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <board.h>
#include <hpm_common.h>
#include <hpm_i2c_drv.h>
#include <hpm_i2c_regs.h>
#include <hpm_soc.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/usb/helper.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"
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

        if (data.payload.size() > kMaxDataLength) {
            publish_error(make_error_view(data));
            return;
        }

        auto* payload = reinterpret_cast<uint8_t*>(write_buffer_.data());
        std::memcpy(payload, data.payload.data(), data.payload.size());
        hpm_stat_t status = status_fail;
        if (data.has_register) {
            uint8_t reg_address = data.reg_address;
            status = i2c_master_address_write(
                i2c_base_, data.slave_address, &reg_address, sizeof(reg_address), payload,
                static_cast<uint32_t>(data.payload.size()));
        } else {
            status = i2c_master_write(
                i2c_base_, data.slave_address, payload, static_cast<uint32_t>(data.payload.size()));
        }

        if (status != status_success) {
            publish_error(make_error_view(data));
            return;
        }
    }

    void handle_downlink_read_config(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kMaxDataLength) {
            publish_error(make_error_view(data));
            return;
        }

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

        if (status != status_success) {
            publish_error(make_error_view(data));
            return;
        }

        publish_read_result(
            data.slave_address, {read_buffer_.data(), data.read_length}, data.has_register,
            data.reg_address);
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

private:
    static constexpr uint16_t kMaxDataLength = (1U << 9) - 1U;
    static constexpr size_t kPendingUplinkQueueSize = 32;
    static constexpr size_t kPendingUplinkQueueMask = kPendingUplinkQueueSize - 1;

    static_assert((kPendingUplinkQueueSize & (kPendingUplinkQueueSize - 1)) == 0);

    enum class UplinkType : uint8_t {
        kReadResult,
        kError,
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

    static uint8_t advance_pending_uplink_index(uint8_t index) {
        return static_cast<uint8_t>((index + 1) & kPendingUplinkQueueMask);
    }

    bool enqueue_read_result(
        uint8_t slave_address, std::span<const std::byte> payload, bool has_register,
        uint8_t reg_address) {
        const utility::InterruptLockGuard guard;
        if (pending_uplink_count_ == kPendingUplinkQueueSize)
            return false;

        auto& entry = pending_uplinks_[pending_uplink_head_];
        entry.type = UplinkType::kReadResult;
        entry.slave_address = slave_address;
        entry.data_length = static_cast<uint16_t>(payload.size());
        entry.has_register = has_register;
        entry.reg_address = reg_address;
        if (!payload.empty()) {
            std::memcpy(entry.payload.data(), payload.data(), payload.size());
        }

        pending_uplink_head_ = advance_pending_uplink_index(pending_uplink_head_);
        pending_uplink_count_ = static_cast<uint8_t>(pending_uplink_count_ + 1);
        return true;
    }

    bool enqueue_error(const data::I2cErrorView& error) {
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

    bool publish_read_result(
        uint8_t slave_address, std::span<const std::byte> payload, bool has_register,
        uint8_t reg_address) {
        if (has_pending_uplink())
            return enqueue_read_result(slave_address, payload, has_register, reg_address);

        const data::I2cDataView data_view{
            .slave_address = slave_address,
            .payload = payload,
            .has_register = has_register,
            .reg_address = reg_address,
        };
        const auto result = usb::get_serializer().write_i2c_read_result(data_id_, data_view);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess)
            return true;

        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        return enqueue_read_result(slave_address, payload, has_register, reg_address);
    }

    bool publish_error(const data::I2cErrorView& error) {
        if (has_pending_uplink())
            return enqueue_error(error);

        const auto result = usb::get_serializer().write_i2c_error(data_id_, error);
        if (result == core::protocol::Serializer::SerializeResult::kSuccess)
            return true;

        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        return enqueue_error(error);
    }

    const data::DataId data_id_;
    I2C_Type* i2c_base_;
    std::array<std::byte, kMaxDataLength> write_buffer_{};
    std::array<std::byte, kMaxDataLength> read_buffer_{};
    uint8_t pending_uplink_head_{0};
    uint8_t pending_uplink_tail_{0};
    uint8_t pending_uplink_count_{0};
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
