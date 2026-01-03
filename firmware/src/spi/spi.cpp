#include "firmware/src/spi/spi.hpp"

#include <cstdint>

#include <board.h>
#include <hpm_spi_drv.h>

namespace librmcs::firmware::spi {

SDK_DECLARE_EXT_ISR_M(IRQn_SPI2, spi2_isr)
void spi2_isr() {
    SPI_Type* base = HPM_SPI2;
    uint32_t flags = spi_get_interrupt_status(base);

    if (!flags) [[unlikely]]
        return;

    if (flags & spi_end_int)
        spi2->transmit_receive_completed_callback();

    spi_clear_interrupt_status(base, flags);
}

} // namespace librmcs::firmware::spi
