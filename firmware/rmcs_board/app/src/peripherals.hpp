#pragma once

#include <cstdint>

#include <librmcs/data/datas.hpp>

namespace librmcs::firmware {

class Peripherals {
public:
    static void init();
    static void poll();

    static bool write_can(data::DataId id, const data::CanDataView& data);
    static bool write_uart(data::DataId id, const data::UartDataView& data);
    static bool configure_uart(data::DataId id, const data::UartConfigView& data);
    static bool write_gpio_digital(uint8_t channel_index, const data::GpioDigitalDataView& data);
    static bool write_gpio_analog(uint8_t channel_index, const data::GpioAnalogDataView& data);
    static bool configure_gpio_digital(uint8_t channel_index, const data::GpioReadConfigView& data);
};

} // namespace librmcs::firmware
