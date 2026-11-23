#pragma once

#include <hpm_dmamux_src.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_uart_drv.h>

#include <librmcs/data/datas.hpp>

#include "board_app.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"

namespace librmcs::firmware::board {

#ifdef BOARD_UART_DBUS
inline constexpr uart::HardwareConfig kDbusConfig = {
    .base = BOARD_UART_DBUS(HPM_UART, _BASE),
    .irq_num = BOARD_UART_DBUS(IRQn_UART, ),
    .dma_src_tx = BOARD_UART_DBUS(HPM_DMA_SRC_UART, _TX),
    .dma_src_rx = BOARD_UART_DBUS(HPM_DMA_SRC_UART, _RX),
};

inline constinit uart::Uart::Lazy uart_dbus{
    data::DataId::kUartDbus, kDbusConfig, 100000, parity_even};
#endif

inline constexpr uart::HardwareConfig kUartConfigs[] = {
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

inline constinit uart::Uart::Lazy uart_array[]{
    uart::Uart::Lazy{data::DataId::kUart0, kUartConfigs[0], 115200, parity_none},
    uart::Uart::Lazy{data::DataId::kUart1, kUartConfigs[1], 115200, parity_none},
#ifdef BOARD_UART2
    uart::Uart::Lazy{data::DataId::kUart2, kUartConfigs[2], 115200, parity_none},
#endif
#ifdef BOARD_UART3
    uart::Uart::Lazy{data::DataId::kUart3, kUartConfigs[3], 115200, parity_none},
#endif
};

} // namespace librmcs::firmware::board
