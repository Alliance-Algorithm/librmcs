#include "firmware/rmcs_board/app/src/i2c/i2c.hpp"

#include <cstdint>

namespace librmcs::firmware::i2c {

void i2c_dma_complete_callback(uint32_t channel) {
    (void)channel;

    if (i2c0)
        i2c0->dma_complete_callback();
}

} // namespace librmcs::firmware::i2c
