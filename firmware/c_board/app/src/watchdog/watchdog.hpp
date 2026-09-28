#pragma once

#include <cstdint>

#include <main.h>
#include <stm32f4xx_ll_iwdg.h>

#include "firmware/c_board/app/src/utility/lazy.hpp"

// IWDG reload value override for targets with longer IRQ-disabled sections (e.g. bootloader
// flash erase of 128KB sectors, worst case seconds). One tick is 2ms (LSI 32kHz / 64).
#ifndef LIBRMCS_WATCHDOG_RELOAD
# define LIBRMCS_WATCHDOG_RELOAD 250U
#endif

namespace librmcs::firmware::watchdog {

class Watchdog {
public:
    using Lazy = utility::Lazy<Watchdog>;

    Watchdog() {
        DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

        LL_IWDG_Enable(IWDG);
        LL_IWDG_EnableWriteAccess(IWDG);
        LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
        LL_IWDG_SetReloadCounter(IWDG, kReloadValue);
        while (LL_IWDG_IsReady(IWDG) == 0U) {}
        LL_IWDG_ReloadCounter(IWDG);
    }

    static void feed() { LL_IWDG_ReloadCounter(IWDG); }

private:
    static constexpr uint32_t kReloadValue = LIBRMCS_WATCHDOG_RELOAD;
};

inline constinit Watchdog::Lazy watchdog;

} // namespace librmcs::firmware::watchdog
