#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <hpm_common.h>
#include <hpm_interrupt.h>
#include <hpm_soc_ip_feature.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/buzzer/status_buzzer.hpp"
#include "firmware/rmcs_board/app/src/led/led.hpp"
#include "firmware/rmcs_board/app/src/peripherals.hpp"
#include "firmware/rmcs_board/app/src/uart/rx_buffer.hpp"
#include "firmware/rmcs_board/app/src/uart/tx_buffer.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::uart {

struct HardwareConfig {
    uint32_t base;
    uint32_t irq_num;
    uint32_t dma_src_tx;
    uint32_t dma_src_rx;
};

class Uart
    : private core::utility::Immovable
    , private TxBuffer
    , private RxBuffer<Uart> {
    friend class RxBuffer<Uart>;

public:
    using Lazy = utility::Lazy<Uart, data::DataId, HardwareConfig, uint32_t, parity_setting_t>;

    explicit Uart(
        data::DataId data_id, HardwareConfig board_config, uint32_t baudrate,
        parity_setting_t parity)
        : TxBuffer(reinterpret_cast<UART_Type*>(board_config.base), board_config.dma_src_tx)
        , RxBuffer(reinterpret_cast<UART_Type*>(board_config.base), board_config.dma_src_rx)
        , data_id_(data_id)
        , uart_base_(reinterpret_cast<UART_Type*>(board_config.base))
        , uart_clock_hz_(init_uart(board_config.irq_num, baudrate, parity)) {}

    bool handle_config(const data::UartConfigView& data) {
        if (!data.baudrate.has_value() || *data.baudrate == 0) [[unlikely]]
            return false;

        TxBuffer::abort_transmit();

        uart_set_baudrate(uart_base_, *data.baudrate, uart_clock_hz_);
        uart_base_->LCR &= ~UART_LCR_DLAB_MASK;
        return true;
    }

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data)) {
            led::led->downlink_buffer_full();
            buzzer::StatusBuzzer::downlink_buffer_full();
        }
    }

    void try_transmit() { TxBuffer::try_dequeue(); }

    void irq_handler() {
        if (uart_is_rxline_idle(uart_base_)) {
            uart_clear_rxline_idle_flag(uart_base_);
            RxBuffer::rx_idle_callback();
        }
    }

private:
    [[nodiscard]] uint32_t init_uart(uint32_t irq_num, uint32_t baudrate, parity_setting_t parity) {
        const uint32_t uart_clock = init_hardware(uart_base_);

        uart_config_t config{};
        uart_default_config(uart_base_, &config);
        config.fifo_enable = true;
        config.dma_enable = true;
        config.src_freq_in_hz = uart_clock;
        config.tx_fifo_level = uart_tx_fifo_trg_not_full;
        config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
        config.baudrate = baudrate;
        config.parity = static_cast<uint8_t>(parity);

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
        return uart_clock;
    }

    static uint32_t init_hardware(UART_Type* ptr);

    void handle_uplink(
        std::span<const std::byte> payload, std::span<const std::byte> payload2, bool is_idle) {
        auto& serializer = Peripherals::uplink_serializer();
        core::utility::assert_debug(
            serializer.write_uart(
                data_id_, {.uart_data = payload, .idle_delimited = is_idle}, payload2)
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    const data::DataId data_id_;
    UART_Type* uart_base_;
    uint32_t uart_clock_hz_;
};

} // namespace librmcs::firmware::uart
