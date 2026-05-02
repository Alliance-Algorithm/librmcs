#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <hpm_common.h>
#include <hpm_soc.h>
#include <hpm_soc_ip_feature.h>
#include <hpm_uart_drv.h>
#include <hpm_uart_regs.h>

#include "board_app.hpp"
#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/led/led.hpp"
#include "firmware/rmcs_board/app/src/uart/rx_buffer.hpp"
#include "firmware/rmcs_board/app/src/uart/tx_buffer.hpp"
#include "firmware/rmcs_board/app/src/usb/helper.hpp"
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
        , uart_base_(reinterpret_cast<UART_Type*>(board_config.base)) {
        init_uart(board_config.irq_num, baudrate, parity);
    }

    void handle_downlink(const data::UartDataView& data) {
        if (!TxBuffer::try_enqueue(data))
            led::led->downlink_buffer_full();
    }

    void try_transmit() { TxBuffer::try_dequeue(); }

    void irq_handler() {
        if (uart_is_rxline_idle(uart_base_)) {
            uart_clear_rxline_idle_flag(uart_base_);
            RxBuffer::rx_idle_callback();
        }
    }

private:
    void init_uart(uint32_t irq_num, uint32_t baudrate, parity_setting_t parity) {
        const uint32_t uart_clock = board::init_uart(uart_base_);

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

constexpr HardwareConfig kDbusBoardConfig = {
    .base = BOARD_UART_DBUS(HPM_UART, _BASE),
    .irq_num = BOARD_UART_DBUS(IRQn_UART, ),
    .dma_src_tx = BOARD_UART_DBUS(HPM_DMA_SRC_UART, _TX),
    .dma_src_rx = BOARD_UART_DBUS(HPM_DMA_SRC_UART, _RX),
};

constexpr HardwareConfig kBoardConfigs[] = {
    {.base = BOARD_UART0(HPM_UART, _BASE),
     .irq_num = BOARD_UART0(IRQn_UART, ),
     .dma_src_tx = BOARD_UART0(HPM_DMA_SRC_UART, _TX),
     .dma_src_rx = BOARD_UART0(HPM_DMA_SRC_UART, _RX)},
    {.base = BOARD_UART1(HPM_UART, _BASE),
     .irq_num = BOARD_UART1(IRQn_UART, ),
     .dma_src_tx = BOARD_UART1(HPM_DMA_SRC_UART, _TX),
     .dma_src_rx = BOARD_UART1(HPM_DMA_SRC_UART, _RX)},
#ifdef BOARD_UART2
    {.base = BOARD_UART2(HPM_UART, _BASE),
     .irq_num = BOARD_UART2(IRQn_UART, ),
     .dma_src_tx = BOARD_UART2(HPM_DMA_SRC_UART, _TX),
     .dma_src_rx = BOARD_UART2(HPM_DMA_SRC_UART, _RX)},
#endif
#ifdef BOARD_UART3
    {.base = BOARD_UART3(HPM_UART, _BASE),
     .irq_num = BOARD_UART3(IRQn_UART, ),
     .dma_src_tx = BOARD_UART3(HPM_DMA_SRC_UART, _TX),
     .dma_src_rx = BOARD_UART3(HPM_DMA_SRC_UART, _RX)},
#endif
};

inline constinit Uart::Lazy uart_dbus{
    data::DataId::kUartDbus, kDbusBoardConfig, 100000, parity_even};

inline constinit Uart::Lazy uart_array[]{
    Uart::Lazy{data::DataId::kUart0, kBoardConfigs[0], 115200, parity_none},
    Uart::Lazy{data::DataId::kUart1, kBoardConfigs[1], 115200, parity_none},
#ifdef BOARD_UART2
    Uart::Lazy{data::DataId::kUart2, kBoardConfigs[2], 115200, parity_none},
#endif
#ifdef BOARD_UART3
    Uart::Lazy{data::DataId::kUart3, kBoardConfigs[3], 115200, parity_none},
#endif
};

constexpr size_t kUartCount = std::size(uart_array);

} // namespace librmcs::firmware::uart
