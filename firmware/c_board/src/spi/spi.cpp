#include "firmware/c_board/src/spi/spi.hpp"

#include <spi.h>

namespace librmcs::firmware::spi {

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == &hspi1) {
        spi1->transmit_receive_async_callback();
    }
}

} // namespace librmcs::firmware::spi
