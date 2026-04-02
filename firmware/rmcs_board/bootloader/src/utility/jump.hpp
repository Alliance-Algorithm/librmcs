#pragma once

#include <hpm_csr_regs.h>
#include <hpm_l1c_drv.h>
#include <hpm_soc.h>

#include "firmware/rmcs_board/bootloader/src/flash/layout.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::utility {

[[noreturn]] inline void jump_to_app() {
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    l1c_dc_disable();
    l1c_fence_i();

    using AppEntry = void (*)();
    reinterpret_cast<AppEntry>(flash::kAppEntryAddress)();

    assert_failed_always();
}

} // namespace librmcs::firmware::utility
