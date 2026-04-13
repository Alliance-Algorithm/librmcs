#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

#include <hpm_common.h>
#include <hpm_mcan_drv.h>
#include <hpm_mcan_regs.h>
#include <hpm_mcan_soc.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>

#include "board_app.hpp"
#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/usb/helper.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::can {

struct HardwareConfig {
    uint32_t base;
    uint32_t irq_num;
};

class Can : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Can, data::DataId, HardwareConfig, uint32_t (*const)[], uint32_t>;

    explicit Can(
        data::DataId data_id, HardwareConfig board_config, uint32_t (*const ram_base)[],
        uint32_t ram_size)
        : data_id_(data_id)
        , can_base_(reinterpret_cast<MCAN_Type*>(board_config.base)) {

        const mcan_msg_buf_attr_t attr = {
            .ram_base = reinterpret_cast<uintptr_t>(ram_base),
            .ram_size = ram_size,
        };
        auto status = mcan_set_msg_buf_attr(can_base_, &attr);
        core::utility::assert_always(status == status_success);

        const uint32_t can_source_clock_freq = board::init_can(can_base_);

        mcan_config_t config;
        mcan_get_default_config(can_base_, &config);
        config.baudrate = 1'000'000; // 1Mbps
        config.mode = mcan_mode_normal;
        config.enable_canfd = false;
        config.ram_config.txbuf_dedicated_txbuf_elem_count = 0;
        config.ram_config.txbuf_fifo_or_queue_elem_count = MCAN_TXBUF_SIZE_CAN_DEFAULT;
        config.ram_config.txfifo_or_txqueue_mode = MCAN_TXBUF_OPERATION_MODE_FIFO;

        mcan_init(can_base_, &config, can_source_clock_freq);
        mcan_enable_interrupts(can_base_, MCAN_INT_RXFIFO0_NEW_MSG);
        intc_m_enable_irq_with_priority(board_config.irq_num, 1);
    }

    void handle_downlink(const data::CanDataView& data) {
        mcan_tx_frame_t frame{};
        if (data.is_extended_can_id) {
            frame.use_ext_id = true;
            frame.ext_id = data.can_id;
        } else {
            frame.use_ext_id = false;
            frame.std_id = data.can_id;
        }
        frame.canfd_frame = false;
        frame.rtr = data.is_remote_transmission;

        core::utility::assert_debug(data.can_data.size() <= 8);
        frame.dlc = data.can_data.size();
        if (!data.can_data.empty())
            std::memcpy(frame.data_8, data.can_data.data(), data.can_data.size());

        mcan_transmit_via_txfifo_nonblocking(can_base_, &frame, nullptr);
    }

    void handle_uplink(core::protocol::FieldId field_id, core::protocol::Serializer& serializer) {
        mcan_rx_message_t rx;
        core::utility::assert_always(mcan_read_rxfifo(can_base_, 0, &rx) == status_success);

        data::CanDataView data;
        const size_t data_length = rx.dlc;
        data.is_fdcan = false;
        data.is_extended_can_id = rx.use_ext_id;
        data.is_remote_transmission = rx.rtr;
        data.can_id = data.is_extended_can_id ? rx.ext_id : rx.std_id;
        data.can_data = {reinterpret_cast<const std::byte*>(rx.data_8), data_length};

        core::utility::assert_always(
            serializer.write_can(field_id, data)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    void irq_handler() {
        const uint32_t flags = mcan_get_interrupt_flags(can_base_);

        if (!flags) [[unlikely]]
            return;

        if (flags & MCAN_INT_RXFIFO0_NEW_MSG) [[likely]]
            handle_uplink(data_id_, usb::get_serializer());

        mcan_clear_interrupt_flags(can_base_, flags);
    }

private:
    const data::DataId data_id_;
    MCAN_Type* can_base_;
};

ATTR_PLACE_AT(".ahb_sram")
inline constinit uint32_t can_msg_buffer[4][MCAN_MSG_BUF_SIZE_IN_WORDS]{};
static_assert(MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1);

constexpr HardwareConfig kBoardConfigs[] = {
    {.base = BOARD_CAN0(HPM_MCAN, _BASE), .irq_num = BOARD_CAN0(IRQn_MCAN, )},
    {.base = BOARD_CAN1(HPM_MCAN, _BASE), .irq_num = BOARD_CAN1(IRQn_MCAN, )},
    {.base = BOARD_CAN2(HPM_MCAN, _BASE), .irq_num = BOARD_CAN2(IRQn_MCAN, )},
    {.base = BOARD_CAN3(HPM_MCAN, _BASE), .irq_num = BOARD_CAN3(IRQn_MCAN, )},
};

inline constinit Can::Lazy can_array[]{
    Can::Lazy{data::DataId::kCan0, kBoardConfigs[0], &can_msg_buffer[0], sizeof(can_msg_buffer[0])},
    Can::Lazy{data::DataId::kCan1, kBoardConfigs[1], &can_msg_buffer[1], sizeof(can_msg_buffer[1])},
    Can::Lazy{data::DataId::kCan2, kBoardConfigs[2], &can_msg_buffer[2], sizeof(can_msg_buffer[2])},
    Can::Lazy{data::DataId::kCan3, kBoardConfigs[3], &can_msg_buffer[3], sizeof(can_msg_buffer[3])},
};
constexpr size_t kCanCount = std::size(can_array);

} // namespace librmcs::firmware::can
