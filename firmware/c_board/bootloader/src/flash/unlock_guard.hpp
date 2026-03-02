#pragma once

#include <main.h>

#include "firmware/c_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::flash {

class UnlockGuard {
public:
    UnlockGuard() { utility::assert_always(HAL_FLASH_Unlock() == HAL_OK); }

    UnlockGuard(const UnlockGuard&) = delete;
    UnlockGuard& operator=(const UnlockGuard&) = delete;
    UnlockGuard(UnlockGuard&&) = delete;
    UnlockGuard& operator=(UnlockGuard&&) = delete;

    ~UnlockGuard() { utility::assert_always(HAL_FLASH_Lock() == HAL_OK); }
};

} // namespace librmcs::firmware::flash
