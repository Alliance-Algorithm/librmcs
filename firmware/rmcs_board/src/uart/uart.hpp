#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <board.h>
#include <hpm_common.h>
#include <hpm_dmamux_src.h>
#include <hpm_soc.h>
#include <hpm_soc_ip_feature.h>
#include <hpm_soc_irq.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/src/uart/rx_buffer.hpp"
#include "firmware/rmcs_board/src/uart/tx_buffer.hpp"
#include "firmware/rmcs_board/src/usb/helper.hpp"
#include "firmware/rmcs_board/src/utility/lazy.hpp"

namespace librmcs::firmware::uart {

class Uart
    : private core::utility::Immovable
    , private TxBuffer
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, data::DataId, uintptr_t, uint32_t, uint32_t, uint32_t>;

    explicit Uart(
        data::DataId data_id, uintptr_t uart_base, uint32_t irq_num, uint32_t tx_dmamux_src,
        uint32_t rx_dmamux_src)
        : TxBuffer(reinterpret_cast<UART_Type*>(uart_base), tx_dmamux_src)
        , RxBuffer(reinterpret_cast<UART_Type*>(uart_base), rx_dmamux_src)
        , data_id_(data_id)
        , uart_base_(reinterpret_cast<UART_Type*>(uart_base)) {
        init_uart(irq_num);
    }

    void handle_downlink(const data::UartDataView& data) { TxBuffer::try_enqueue(data); }

    void try_transmit() { TxBuffer::try_dequeue(); }

    void isr() {
        if (uart_is_rxline_idle(uart_base_)) {
            uart_clear_rxline_idle_flag(uart_base_);
            RxBuffer::rx_idle_callback();
        }
    }

private:
    void init_uart(uint32_t irq_num) {
        board_init_uart(uart_base_);
        const uint32_t uart_clock = board_init_uart_clock(uart_base_);

        uart_config_t config{};
        uart_default_config(uart_base_, &config);
        config.fifo_enable = true;
        config.dma_enable = true;
        config.src_freq_in_hz = uart_clock;
        config.tx_fifo_level = uart_tx_fifo_trg_not_full;
        config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
        config.baudrate = 115200;

        static_assert(HPM_IP_FEATURE_UART_TX_IDLE_DETECT == 1);
        config.txidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
        config.txidle_config.detect_enable = true;
        config.txidle_config.threshold = 16;

        static_assert(HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1);
        config.rxidle_config.detect_enable = true;
        config.rxidle_config.detect_irq_enable = true;
        config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
        config.rxidle_config.threshold = 10;

        core::utility::assert_always(uart_init(uart_base_, &config) == status_success);
        intc_m_enable_irq_with_priority(irq_num, 1);
    }

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        auto& serializer = usb::get_serializer();
        core::utility::assert_debug(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    const data::DataId data_id_;
    UART_Type* uart_base_;
};

inline constinit Uart::Lazy uart3{
    data::DataId::UART3, HPM_UART3_BASE, IRQn_UART3, HPM_DMA_SRC_UART3_TX, HPM_DMA_SRC_UART3_RX};

} // namespace librmcs::firmware::uart
