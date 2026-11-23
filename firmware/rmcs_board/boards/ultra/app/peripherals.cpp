#include "firmware/rmcs_board/app/src/peripherals.hpp"

#include <cstdint>

#include <librmcs/data/datas.hpp>

#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/temperature.hpp"
#include "rs485.hpp"

namespace librmcs::firmware {

void Peripherals::init() {
    for (auto& can : can::can_array)
        can.init();
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
    for (auto& port : rs485::ports)
        port.init();
}

void Peripherals::poll() {
    for (auto& port : rs485::ports)
        port->try_transmit();
}

bool Peripherals::configure_uart(data::DataId, const data::UartConfigView&) { return false; }
bool Peripherals::write_uart(data::DataId id, const data::UartDataView& data) {
    switch (id) {
    case data::DataId::kRs4850: rs485::ports[0]->handle_downlink(data); return true;
    case data::DataId::kRs4851: rs485::ports[1]->handle_downlink(data); return true;
    default: return false;
    }
}

bool Peripherals::configure_uart(data::DataId id, const data::UartConfigView& data) {
    switch (id) {
    case data::DataId::kRs4850Config: return rs485::ports[0]->handle_config(data);
    case data::DataId::kRs4851Config: return rs485::ports[1]->handle_config(data);
    default: return false;
    }
}

bool Peripherals::write_gpio_digital(uint8_t, const data::GpioDigitalDataView&) { return false; }

bool Peripherals::write_gpio_analog(uint8_t, const data::GpioAnalogDataView&) { return false; }

bool Peripherals::configure_gpio_digital(uint8_t, const data::GpioReadConfigView&) { return false; }

} // namespace librmcs::firmware
