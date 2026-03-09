#pragma once

#include <cstdint>

#include <main.h>
#include <tim.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::gpio {

class Gpio : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Gpio>;

    Gpio() {
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) == HAL_OK);
        core::utility::assert_always(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) == HAL_OK);

        set_compare(1, 0);
        set_compare(2, 0);
        set_compare(3, 0);
        set_compare(4, 0);
        set_compare(5, 0);
        set_compare(6, 0);
        set_compare(7, 0);
    }

    void handle_digital_write(const data::GpioDigitalDataView& data) {
        if (!is_supported_channel(data.channel))
            return;

        set_compare(data.channel, data.high ? kCounterPeriod : 0);
    }

    void handle_analog_write(const data::GpioAnalogDataView& data) {
        if (!is_supported_channel(data.channel))
            return;

        set_compare(data.channel, duty_to_compare(data.value));
    }

private:
    static constexpr uint32_t kCounterPeriod = 60000;

    static bool is_supported_channel(uint8_t channel) { return channel >= 1 && channel <= 7; }

    static uint32_t duty_to_compare(uint16_t duty) {
        return ((static_cast<uint32_t>(duty) * kCounterPeriod) + 32767U) / 65535U;
    }

    void set_compare(uint8_t channel, uint32_t compare) {
        core::utility::assert_debug_lazy([&]() noexcept { return is_supported_channel(channel); });
        *channel_compares_[channel - 1] = compare;
    }

    volatile uint32_t* channel_compares_[7] = {
        &htim1.Instance->CCR1, &htim1.Instance->CCR2, &htim1.Instance->CCR3, &htim1.Instance->CCR4,
        &htim8.Instance->CCR1, &htim8.Instance->CCR2, &htim8.Instance->CCR3,
    };
};

inline constinit Gpio::Lazy gpio;

} // namespace librmcs::firmware::gpio
