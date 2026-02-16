#pragma once

#include <main.h> // IWYU pragma: keep (disable/enable irq)

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"

namespace librmcs::firmware::utility {

class InterruptMutex {
public:
    InterruptMutex() = delete;

    static void lock() {
        __disable_irq();
        ++lock_count_;
    }

    static void unlock() {
        core::utility::assert_debug(lock_count_ > 0);
        if (--lock_count_ == 0) {
            __enable_irq();
        }
    }

private:
    // Interrupts are disabled during the initialization.
    static inline int lock_count_ = 1;
};

class InterruptLockGuard : private core::utility::Immovable {
public:
    InterruptLockGuard() { InterruptMutex::lock(); }

    InterruptLockGuard(const InterruptLockGuard&) = delete;
    InterruptLockGuard& operator=(const InterruptLockGuard&) = delete;
    InterruptLockGuard(InterruptLockGuard&&) = delete;
    InterruptLockGuard& operator=(InterruptLockGuard&&) = delete;

    ~InterruptLockGuard() { InterruptMutex::unlock(); }
};

} // namespace librmcs::firmware::utility
