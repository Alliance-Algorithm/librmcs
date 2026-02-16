#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <can.h>
#include <stm32f407xx.h>
#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal_def.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/src/led/led.hpp"
#include "firmware/c_board/src/utility/lazy.hpp"
#include "firmware/c_board/src/utility/ring_buffer.hpp"

namespace librmcs::firmware::can {

class Can : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Can, CAN_HandleTypeDef*, uint32_t, uint32_t>;

    Can(CAN_HandleTypeDef* hal_can_handle, uint32_t hal_filter_bank,
        uint32_t hal_slave_start_filter_bank)
        : hal_can_handle_(hal_can_handle) {
        config_can(hal_filter_bank, hal_slave_start_filter_bank);
    }

    void handle_downlink(const data::CanDataView& data) {
        if (data.is_fdcan) [[unlikely]]
            return; // TODO: Support FDCAN when protocol ready

        auto construct = [&data](std::byte* storage) noexcept {
            auto& mailbox = *new (storage) TransmitMailboxData{};

            mailbox.identifier =
                ((data.is_extended_can_id ? data.can_id << CAN_TI0R_EXID_Pos
                                          : data.can_id << CAN_TI0R_STID_Pos)
                 | (data.is_extended_can_id ? CAN_ID_EXT : CAN_ID_STD)
                 | (data.is_remote_transmission ? CAN_RTR_REMOTE : CAN_RTR_DATA) | CAN_TI0R_TXRQ);

            core::utility::assert_debug(data.can_data.size() <= 8);
            mailbox.data_length_and_timestamp = data.can_data.size();
            if (!data.can_data.empty())
                std::memcpy(mailbox.data, data.can_data.data(), data.can_data.size());
        };

        if (!transmit_buffer_.emplace_back_n(construct, 1))
            led::led->downlink_buffer_full();
    }

    void handle_uplink(data::DataId field_id, core::protocol::Serializer& serializer) {
        auto hal_can_state = hal_can_handle_->State;
        auto* hal_can_instance = hal_can_handle_->Instance;

        core::utility::assert_always(
            (hal_can_state == HAL_CAN_STATE_READY) || (hal_can_state == HAL_CAN_STATE_LISTENING));
        core::utility::assert_always((hal_can_instance->RF0R & CAN_RF0R_FMP0) != 0U);

        const auto rir = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
        const auto rdtr = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR;

        data::CanDataView data{};
        data.is_fdcan = false;
        data.is_extended_can_id = static_cast<bool>(CAN_RI0R_IDE & rir);
        data.is_remote_transmission = static_cast<bool>(CAN_RI0R_RTR & rir);

        if (data.is_extended_can_id) {
            data.can_id = ((CAN_RI0R_EXID | CAN_RI0R_STID) & rir) >> CAN_RI0R_EXID_Pos;
        } else {
            data.can_id = (CAN_RI0R_STID & rir) >> CAN_TI0R_STID_Pos;
        }

        size_t can_data_length = (CAN_RDT0R_DLC & rdtr) >> CAN_RDT0R_DLC_Pos;
        if (data.is_remote_transmission)
            can_data_length = 0;

        alignas(uint32_t) std::array<std::byte, 8> can_data{};
        const uint32_t rdlr = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
        const uint32_t rdhr = hal_can_instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
        std::memcpy(can_data.data(), &rdlr, sizeof(uint32_t));
        std::memcpy(can_data.data() + 4, &rdhr, sizeof(uint32_t));
        data.can_data = {can_data.data(), can_data_length};

        core::utility::assert_always(
            serializer.write_can(field_id, data)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);

        hal_can_instance->RF0R |= CAN_RF0R_RFOM0;
    }

    bool try_transmit() {
        auto* hcan = hal_can_handle_;

        auto state = hcan->State;
        core::utility::assert_always(
            (state == HAL_CAN_STATE_READY) || (state == HAL_CAN_STATE_LISTENING));

        const uint32_t tsr = hcan->Instance->TSR;
        auto free_mailbox_count =
            !!(tsr & CAN_TSR_TME0) + !!(tsr & CAN_TSR_TME1) + !!(tsr & CAN_TSR_TME2);

        return transmit_buffer_.pop_front_n(
            [this, hcan](const TransmitMailboxData& mailbox_data) noexcept {
                auto target_mailbox_index =
                    (hcan->Instance->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
                core::utility::assert_always(target_mailbox_index <= 2);

                auto& target_mailbox = hal_can_handle_->Instance->sTxMailBox[target_mailbox_index];
                target_mailbox.TDTR = mailbox_data.data_length_and_timestamp;
                target_mailbox.TDLR = mailbox_data.data[0];
                target_mailbox.TDHR = mailbox_data.data[1];
                target_mailbox.TIR = mailbox_data.identifier;
            },
            free_mailbox_count);
    }

private:
    void config_can(uint32_t hal_filter_bank, uint32_t hal_slave_start_filter_bank) {
        CAN_FilterTypeDef filter_config;

        filter_config.FilterBank = hal_filter_bank;
        filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
        filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
        filter_config.FilterIdHigh = 0x0000;
        filter_config.FilterIdLow = 0x0000;
        filter_config.FilterMaskIdHigh = 0x0000;
        filter_config.FilterMaskIdLow = 0x0000;
        filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter_config.FilterActivation = CAN_FILTER_ENABLE;
        filter_config.SlaveStartFilterBank = hal_slave_start_filter_bank;

        constexpr auto ok = HAL_OK;
        core::utility::assert_always(HAL_CAN_ConfigFilter(hal_can_handle_, &filter_config) == ok);
        core::utility::assert_always(HAL_CAN_Start(hal_can_handle_) == ok);
        core::utility::assert_always(
            HAL_CAN_ActivateNotification(hal_can_handle_, CAN_IT_RX_FIFO0_MSG_PENDING) == ok);
    }

    CAN_HandleTypeDef* hal_can_handle_;

    struct TransmitMailboxData {
        uint32_t identifier;                // CAN_TxMailBox_TypeDef::TIR
        uint32_t data_length_and_timestamp; // CAN_TxMailBox_TypeDef::TDTR
        uint32_t data[2];                   // CAN_TxMailBox_TypeDef::TDLR & TDHR
    };
    utility::RingBuffer<TransmitMailboxData, 16> transmit_buffer_;
};

inline constinit Can::Lazy can1{&hcan1, 0, 14};
inline constinit Can::Lazy can2{&hcan2, 14, 14};

} // namespace librmcs::firmware::can
