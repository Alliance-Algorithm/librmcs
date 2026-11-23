#include "firmware/rmcs_board/app/src/peripherals.hpp"

#include <cstdint>

#include <librmcs/data/datas.hpp>
#include <librmcs/spec/gpio.hpp>

#include "board_app.hpp"
#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/rmcs_board/app/src/spi/bmi088/temperature.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"

namespace librmcs::firmware {

void Peripherals::init() {
    for (auto& can : can::can_array)
        can.init();

    uart::uart_dbus.init();
    for (auto& board_uart : uart::uart_array)
        board_uart.init();

    gpio::gpio.init();

    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
    spi::bmi088::temperature.init();
}

void Peripherals::poll() {
    for (auto& board_uart : uart::uart_array)
        board_uart->try_transmit();
    gpio::gpio->poll_periodic_input_samples();
}

bool Peripherals::write_uart(data::DataId id, const data::UartDataView& data) {
    switch (id) {
    case data::DataId::kUart0: uart::uart_array[0]->handle_downlink(data); return true;
    case data::DataId::kUart1: uart::uart_array[1]->handle_downlink(data); return true;
#ifdef BOARD_UART2
    case data::DataId::kUart2: uart::uart_array[2]->handle_downlink(data); return true;
#endif
#ifdef BOARD_UART3
    case data::DataId::kUart3: uart::uart_array[3]->handle_downlink(data); return true;
#endif
    default: return false;
    }
}

bool Peripherals::configure_uart(data::DataId id, const data::UartConfigView& data) {
    switch (id) {
    case data::DataId::kUartDbusConfig: uart::uart_dbus->handle_config(data); return true;
    case data::DataId::kUart0Config: uart::uart_array[0]->handle_config(data); return true;
    case data::DataId::kUart1Config: uart::uart_array[1]->handle_config(data); return true;
#ifdef BOARD_UART2
    case data::DataId::kUart2Config: uart::uart_array[2]->handle_config(data); return true;
#endif
#ifdef BOARD_UART3
    case data::DataId::kUart3Config: uart::uart_array[3]->handle_config(data); return true;
#endif
    default: return false;
    }
}

bool Peripherals::write_gpio_digital(uint8_t channel_index, const data::GpioDigitalDataView& data) {
    if (data.timestamp_quarter_us.has_value())
        return false;
    if (channel_index >= board::spec::kGpioDescriptors.size())
        return false;

    const auto& gpio_descriptor = board::spec::kGpioDescriptors[channel_index];
    if (!gpio_descriptor.supports(spec::GpioCapability::kDigitalWrite))
        return false;

    gpio::gpio->handle_digital_write(channel_index, data);
    return true;
}

bool Peripherals::write_gpio_analog(uint8_t channel_index, const data::GpioAnalogDataView& data) {
    if (channel_index >= board::spec::kGpioDescriptors.size())
        return false;

    const auto& gpio_descriptor = board::spec::kGpioDescriptors[channel_index];
    if (!gpio_descriptor.supports(spec::GpioCapability::kAnalogWrite))
        return false;

    gpio::gpio->handle_analog_write(channel_index, data);
    return true;
}

bool Peripherals::configure_gpio_digital(
    uint8_t channel_index, const data::GpioReadConfigView& data) {
    if (channel_index >= board::spec::kGpioDescriptors.size())
        return false;

    const auto& gpio_descriptor = board::spec::kGpioDescriptors[channel_index];
    if (!data.supported(gpio_descriptor))
        return false;

    gpio::gpio->handle_digital_read(channel_index, data);
    return true;
}

} // namespace librmcs::firmware
