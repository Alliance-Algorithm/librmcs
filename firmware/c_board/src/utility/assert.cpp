#include "core/src/utility/assert.hpp"

#include <source_location>

#include <main.h>

#include "firmware/c_board/src/utility/interrupt_lock.hpp"

namespace librmcs::core::utility {

const char* volatile assert_file = nullptr;
volatile unsigned int assert_line = 0;
const char* volatile assert_function = nullptr;

namespace {
inline void force_led_red() noexcept {
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {};
    gpio_init.Pin = LED_R_Pin | LED_G_Pin | LED_B_Pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &gpio_init);

    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}
} // namespace

[[noreturn]] void assert_func(const std::source_location& location) {
    firmware::utility::InterruptMutex::lock();

    assert_file = location.file_name();
    assert_line = location.line();
    assert_function = location.function_name();

    force_led_red();

    __builtin_trap();
}

} // namespace librmcs::core::utility
