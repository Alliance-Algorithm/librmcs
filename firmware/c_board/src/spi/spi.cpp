#include "firmware/c_board/src/spi/spi.hpp"

#include <spi.h>

#ifndef NDEBUG
# include "core/src/utility/assert.hpp"
#endif

namespace librmcs::firmware::spi {

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == &hspi1) {
        spi1->transmit_receive_async_callback(true);
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
#ifndef NDEBUG
    // Fail-fast in debug builds.
    core::utility::assert_failed_always();
#endif
    // Release fallback: cleanup (drop this frame, deassert CS, and release the lock).

    if (hspi == &hspi1) {
        spi1->transmit_receive_async_callback(false);
    }
}

} // namespace librmcs::firmware::spi
