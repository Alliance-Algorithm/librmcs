#pragma once

#include <hpm_common.h>
#include <hpm_dmamux_src.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include <librmcs/data/datas.hpp>

#include "board_app.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"

namespace librmcs::firmware::rs485 {

inline constexpr uart::HardwareConfig kConfigs[]{
    {.base = BOARD_RS4850(HPM_UART, _BASE),
     .irq_num = BOARD_RS4850(IRQn_UART, ),
     .dma_src_tx = BOARD_RS4850(HPM_DMA_SRC_UART, _TX),
     .dma_src_rx = BOARD_RS4850(HPM_DMA_SRC_UART, _RX)},
    {.base = BOARD_RS4851(HPM_UART, _BASE),
     .irq_num = BOARD_RS4851(IRQn_UART, ),
     .dma_src_tx = BOARD_RS4851(HPM_DMA_SRC_UART, _TX),
     .dma_src_rx = BOARD_RS4851(HPM_DMA_SRC_UART, _RX)},
};

inline constinit uart::Uart::Lazy ports[]{
    uart::Uart::Lazy{data::DataId::kRs4850, kConfigs[0], 115200, parity_none},
    uart::Uart::Lazy{data::DataId::kRs4851, kConfigs[1], 115200, parity_none},
};

} // namespace librmcs::firmware::rs485
