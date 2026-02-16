#include <cstdint>

#include <main.h>
#include <stm32f4xx_hal_gpio.h>

#include "firmware/c_board/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/src/spi/bmi088/gyro.hpp"

namespace librmcs::firmware::gpio {

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
    if (gpio_pin == INT1_ACC_Pin) {
        spi::bmi088::accelerometer->data_ready_callback();
    } else if (gpio_pin == INT1_GYRO_Pin) {
        spi::bmi088::gyroscope->data_ready_callback();
    }
}

} // namespace librmcs::firmware::gpio
