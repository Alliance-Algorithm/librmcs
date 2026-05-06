#include "firmware/c_board/app/src/gpio/gpio.hpp"

#include <cstdint>

#include <gpio.h>
#include <main.h>

#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"

namespace librmcs::firmware::gpio {

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin) {
    if (gpio_pin == INT1_ACC_Pin) {
        const uint32_t capture_timestamp_quarter_us =
            timer::timer->timepoint().time_since_epoch().count();
        spi::bmi088::accelerometer->data_ready_callback(capture_timestamp_quarter_us);
    } else if (gpio_pin == INT1_GYRO_Pin) {
        const uint32_t capture_timestamp_quarter_us =
            timer::timer->timepoint().time_since_epoch().count();
        spi::bmi088::gyroscope->data_ready_callback(capture_timestamp_quarter_us);
    } else {
        gpio::gpio->handle_input_edge_interrupt(gpio_pin);
    }
}

} // namespace librmcs::firmware::gpio
