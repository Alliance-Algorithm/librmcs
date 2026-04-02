#pragma once

#include <cstdint>

#include <main.h>

#include "firmware/c_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::utility {

inline void jump_to_app(uint32_t app_address) {
    const uint32_t app_stack = *reinterpret_cast<volatile uint32_t*>(app_address);
    const uint32_t app_entry = *reinterpret_cast<volatile uint32_t*>(app_address + 4U);
    const auto app_reset_handler = reinterpret_cast<void (*)()>(app_entry);

    __disable_irq();

    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (uint32_t i = 0; i < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

    __set_BASEPRI(0U);
    __set_FAULTMASK(0U);
    __set_CONTROL(0U);
    __set_PSP(0U);

    SCB->VTOR = app_address;
    __DSB();
    __ISB();

    __set_MSP(app_stack);
    __set_PRIMASK(0U);

    __DSB();
    __ISB();

    app_reset_handler();
    assert_failed_always();
}

} // namespace librmcs::firmware::utility
