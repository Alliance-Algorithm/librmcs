#pragma once

#include <cstdint>

#include <board.h>

#include "core/src/utility/immovable.hpp"

namespace librmcs::firmware::utility {

class InterruptLockGuard : core::utility::Immovable {
public:
    InterruptLockGuard() noexcept
        : flags_(disable_global_irq(CSR_MSTATUS_MIE_MASK)) {}

    ~InterruptLockGuard() noexcept { restore_global_irq(flags_ & CSR_MSTATUS_MIE_MASK); }

private:
    uint32_t flags_;
};

} // namespace librmcs::firmware::utility
