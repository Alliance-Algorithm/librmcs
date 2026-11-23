#include "firmware/rmcs_board/app/src/peripherals.hpp"

#include <cstdint>

#include <librmcs/data/datas.hpp>

#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/temperature.hpp"

namespace librmcs::firmware {

void Peripherals::init() {
    for (auto& can : can::can_array)
        can.init();
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
}

void Peripherals::poll() {}

bool Peripherals::write_uart(data::DataId, const data::UartDataView&) { return false; }

bool Peripherals::configure_uart(data::DataId, const data::UartConfigView&) { return false; }

bool Peripherals::write_gpio_digital(uint8_t, const data::GpioDigitalDataView&) { return false; }

bool Peripherals::write_gpio_analog(uint8_t, const data::GpioAnalogDataView&) { return false; }

bool Peripherals::configure_gpio_digital(uint8_t, const data::GpioReadConfigView&) { return false; }

} // namespace librmcs::firmware
