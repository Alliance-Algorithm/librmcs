#include "firmware/rmcs_board/src/spi/spi.hpp"

#include <cstdint>

#include <hpm_soc.h>
#include <hpm_soc_irq.h>
#include <hpm_spi_drv.h>
#include <hpm_spi_regs.h>

namespace librmcs::firmware::spi {

SDK_DECLARE_EXT_ISR_M(IRQn_SPI2, spi2_isr)
void spi2_isr() {
    SPI_Type* base = HPM_SPI2;
    const uint32_t flags = spi_get_interrupt_status(base);

    if (!flags) [[unlikely]]
        return;

    if (flags & spi_end_int)
        spi2->transmit_receive_async_callback();

    spi_clear_interrupt_status(base, flags);
}

} // namespace librmcs::firmware::spi
