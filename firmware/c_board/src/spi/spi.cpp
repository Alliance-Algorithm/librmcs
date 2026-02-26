#include "firmware/c_board/src/spi/spi.hpp"

#include <main.h>
#include <spi.h>

#include "core/src/utility/assert.hpp"

namespace librmcs::firmware::spi {

void Spi::dma_transfer_complete_callback_global(DMA_HandleTypeDef* hal_dma_handle) {
    if (hal_dma_handle->Parent == &hspi1) {
        spi1->dma_transfer_complete_callback();
    }
}

void Spi::dma_error_callback_global(DMA_HandleTypeDef* hal_dma_handle) {
    if (hal_dma_handle->Parent == &hspi1) {
        spi1->transmit_receive_async_callback(false);
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef*) {
    core::utility::assert_failed_always();
}

// NOLINTNEXTLINE(readability-inconsistent-declaration-parameter-name)
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hal_spi_handle) {
    if (hal_spi_handle == &hspi1) {
        spi1->transmit_receive_async_callback(false);
    }
}

} // namespace librmcs::firmware::spi
