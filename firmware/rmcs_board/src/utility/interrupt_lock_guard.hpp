#pragma once

#include <cstdint>

#include <hpm_csr_regs.h>
#include <hpm_soc.h>

#include "core/src/utility/immovable.hpp"

namespace librmcs::firmware::utility {

class InterruptLockGuard : core::utility::Immovable {
public:
    InterruptLockGuard() noexcept
        : flags_(disable_global_irq(CSR_MSTATUS_MIE_MASK)) {}

    InterruptLockGuard(const InterruptLockGuard&) = delete;
    InterruptLockGuard& operator=(const InterruptLockGuard&) = delete;
    InterruptLockGuard(InterruptLockGuard&&) = delete;
    InterruptLockGuard& operator=(InterruptLockGuard&&) = delete;

    ~InterruptLockGuard() noexcept { restore_global_irq(flags_ & CSR_MSTATUS_MIE_MASK); }

private:
    uint32_t flags_;
};

} // namespace librmcs::firmware::utility
