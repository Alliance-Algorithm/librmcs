#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <board.h>
#include <hpm_common.h>
#include <hpm_mcan_drv.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/src/utility/lazy.hpp"

namespace librmcs::firmware::can {

class Can : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Can, uintptr_t, uint32_t, uint32_t (*const)[], uint32_t>;

    explicit Can(
        uintptr_t can_base, uint32_t irq_num, uint32_t (*const ram_base)[], uint32_t ram_size)
        : can_base_(reinterpret_cast<MCAN_Type*>(can_base)) {

        mcan_msg_buf_attr_t attr = {reinterpret_cast<uintptr_t>(ram_base), ram_size};
        auto status = mcan_set_msg_buf_attr(can_base_, &attr);
        core::utility::assert_always(status == status_success);

        board_init_can(can_base_);
        uint32_t can_source_clock_freq = board_init_can_clock(can_base_);

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
        intc_m_enable_irq_with_priority(irq_num, 1);
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
        size_t data_length = rx.dlc;
        data.is_fdcan = false;
        data.is_extended_can_id = rx.use_ext_id;
        data.is_remote_transmission = rx.rtr;
        data.can_id = data.is_extended_can_id ? rx.ext_id : rx.std_id;
        data.can_data = {reinterpret_cast<const std::byte*>(rx.data_8), data_length};

        core::utility::assert_always(
            serializer.write_can(field_id, data)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

private:
    MCAN_Type* can_base_;
};

ATTR_PLACE_AT(".ahb_sram") inline uint32_t can0_msg_buffer[MCAN_MSG_BUF_SIZE_IN_WORDS];
inline constinit Can::Lazy can0{
    HPM_MCAN0_BASE, IRQn_MCAN0, &can0_msg_buffer, sizeof(can0_msg_buffer)};

ATTR_PLACE_AT(".ahb_sram") inline uint32_t can1_msg_buffer[MCAN_MSG_BUF_SIZE_IN_WORDS];
inline constinit Can::Lazy can1{
    HPM_MCAN1_BASE, IRQn_MCAN1, &can1_msg_buffer, sizeof(can1_msg_buffer)};

ATTR_PLACE_AT(".ahb_sram") inline uint32_t can2_msg_buffer[MCAN_MSG_BUF_SIZE_IN_WORDS];
inline constinit Can::Lazy can2{
    HPM_MCAN2_BASE, IRQn_MCAN2, &can2_msg_buffer, sizeof(can2_msg_buffer)};

ATTR_PLACE_AT(".ahb_sram") inline uint32_t can3_msg_buffer[MCAN_MSG_BUF_SIZE_IN_WORDS];
inline constinit Can::Lazy can3{
    HPM_MCAN3_BASE, IRQn_MCAN3, &can3_msg_buffer, sizeof(can3_msg_buffer)};

static_assert(MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1);

} // namespace librmcs::firmware::can
