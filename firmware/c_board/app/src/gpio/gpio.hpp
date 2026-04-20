#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>

#include <gpio.h>
#include <main.h>
#include <tim.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/include/librmcs/spec/c_board/gpio.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/timer/timer.hpp"
#include "firmware/c_board/app/src/usb/helper.hpp"
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

        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

        constexpr data::GpioReadConfigView k_default_input_config{};
        for (const auto& gpio : spec::c_board::kGpioDescriptors) {
            set_pwm_compare(gpio.channel_index, 0);
            configure_digital_input_mode(gpio.channel_index, k_default_input_config);
        }
    }

    void handle_digital_write(uint8_t channel_index, const data::GpioDigitalDataView& data) {
        configure_output_mode(channel_index);
        set_pwm_compare(channel_index, data.high ? kPwmCounterPeriod : 0);
    }

    void handle_analog_write(uint8_t channel_index, const data::GpioAnalogDataView& data) {
        configure_output_mode(channel_index);
        set_pwm_compare(channel_index, duty16_to_pwm_compare(data.value));
    }

    void handle_digital_read(uint8_t channel_index, const data::GpioReadConfigView& data) {
        configure_digital_input_mode(channel_index, data);
    }

    void poll_periodic_input_samples() {
        const auto now = timer::timer->timepoint();

        for (std::size_t channel_index = 0; channel_index < kChannelCount; ++channel_index) {
            auto& state = channel_states_[channel_index];
            if (state.mode != GpioMode::kDigitalInput || state.period == kNoPeriod)
                continue;
            if (!timer::timer->check_expired(state.next_sample_time, state.period))
                continue;

            publish_digital_input_sample(static_cast<uint8_t>(channel_index));
            state.next_sample_time = now;
        }
    }

    void handle_input_edge_interrupt(uint16_t gpio_pin) {
        const uint8_t channel_index = channel_index_from_exti_line(exti_line_from_pin(gpio_pin));
        if (channel_index == kInvalidChannelIndex)
            return;

        auto& state = channel_state(channel_index);
        if (state.mode != GpioMode::kDigitalInput || (!state.rising_edge && !state.falling_edge))
            return;

        publish_digital_input_sample(channel_index);
    }

private:
    enum class GpioMode : uint8_t { kOutput = 0, kDigitalInput = 1, kAnalogInput = 2 };

    struct ChannelState {
        GpioMode mode = GpioMode::kOutput;
        bool rising_edge = false;
        bool falling_edge = false;
        data::GpioPull pull = data::GpioPull::kNone;
        timer::Timer::Duration period = timer::Timer::Duration::zero();
        timer::Timer::TimePoint next_sample_time;
    };

    struct ChannelHardware {
        GPIO_TypeDef* gpio_port;
        uint16_t gpio_pin;
        uint8_t alternate_function;
        volatile uint32_t* compare_register;
    };

    static constexpr uint32_t kPwmCounterPeriod = 60000;
    static constexpr auto kNoPeriod = timer::Timer::Duration::zero();
    static constexpr std::size_t kChannelCount = std::size(spec::c_board::kGpioDescriptors);
    static constexpr uint8_t kInvalidChannelIndex = 0xFFU;

    void configure_output_mode(uint8_t channel_index) {
        auto& state = channel_state(channel_index);
        if (state.mode == GpioMode::kOutput)
            return;

        state.mode = GpioMode::kOutput;
        configure_hal_gpio_output(channel_index);
    }

    void configure_digital_input_mode(uint8_t channel_index, const data::GpioReadConfigView& data) {
        auto& state = channel_state(channel_index);

        const auto& asap = data.asap;
        auto rising_edge = data.rising_edge;
        auto falling_edge = data.falling_edge;
        const auto pull = data.pull;
        const auto period =
            (data.period_ms == 0)
                ? kNoPeriod
                : timer::Timer::to_duration_checked(std::chrono::milliseconds{data.period_ms});

        if (state.mode != GpioMode::kDigitalInput //
            || state.rising_edge != rising_edge || state.falling_edge != falling_edge
            || state.pull != pull || state.period != period) {

            state.mode = GpioMode::kDigitalInput;
            state.rising_edge = rising_edge;
            state.falling_edge = falling_edge;
            state.pull = pull;
            state.period = period;
            state.next_sample_time = timer::timer->timepoint();

            configure_hal_gpio_input(channel_index, rising_edge, falling_edge, pull);
        }

        if (asap)
            publish_digital_input_sample(channel_index);
    }

    void set_pwm_compare(uint8_t channel_index, uint32_t compare) {
        const auto& hardware = channel_hardware(channel_index);
        *hardware.compare_register = compare;
    }

    void configure_hal_gpio_output(uint8_t channel_index) {
        const auto& hardware = channel_hardware(channel_index);

        GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = hardware.gpio_pin;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init.Alternate = hardware.alternate_function;
        HAL_GPIO_Init(hardware.gpio_port, &gpio_init);
    }

    void configure_hal_gpio_input(
        uint8_t channel_index, bool rising_edge, bool falling_edge, data::GpioPull pull) {
        const auto& hardware = channel_hardware(channel_index);

        GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = hardware.gpio_pin;
        if (rising_edge && falling_edge) {
            gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
        } else if (rising_edge) {
            gpio_init.Mode = GPIO_MODE_IT_RISING;
        } else if (falling_edge) {
            gpio_init.Mode = GPIO_MODE_IT_FALLING;
        } else {
            gpio_init.Mode = GPIO_MODE_INPUT;
        }
        gpio_init.Pull = hal_gpio_pull(pull);
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(hardware.gpio_port, &gpio_init);
    }

    void publish_digital_input_sample(uint8_t channel_index) {
        const auto& hardware = channel_hardware(channel_index);
        const bool high = HAL_GPIO_ReadPin(hardware.gpio_port, hardware.gpio_pin) == GPIO_PIN_SET;

        auto& serializer = usb::get_serializer();
        core::utility::assert_debug(
            serializer.write_gpio_digital_read_result(channel_index, {.high = high})
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    static uint8_t exti_line_from_pin(uint16_t gpio_pin) {
        core::utility::assert_debug(gpio_pin != 0);

        uint8_t line = 0;
        while ((gpio_pin & 0x1U) == 0U) {
            gpio_pin >>= 1;
            ++line;
        }

        return line;
    }

    static uint8_t channel_index_from_exti_line(uint8_t exti_line) {
        switch (exti_line) {
        case 9: return spec::c_board::kGpioDescriptors.kPwm1.channel_index;
        case 11: return spec::c_board::kGpioDescriptors.kPwm2.channel_index;
        case 13: return spec::c_board::kGpioDescriptors.kPwm3.channel_index;
        case 14: return spec::c_board::kGpioDescriptors.kPwm4.channel_index;
        case 6: return spec::c_board::kGpioDescriptors.kPwm5.channel_index;
        case 7: return spec::c_board::kGpioDescriptors.kPwm7.channel_index;
        default: return kInvalidChannelIndex;
        }
    }

    ChannelState& channel_state(uint8_t channel_index) {
        const auto index = static_cast<std::size_t>(channel_index);
        core::utility::assert_debug(index < kChannelCount);
        return channel_states_[index];
    }

    const ChannelHardware& channel_hardware(uint8_t channel_index) const {
        const auto index = static_cast<std::size_t>(channel_index);
        core::utility::assert_debug(index < kChannelCount);
        return channel_hardware_[index];
    }

    static uint32_t duty16_to_pwm_compare(uint16_t duty) {
        return ((static_cast<uint32_t>(duty) * kPwmCounterPeriod) + 32767U) / 65535U;
    }

    static uint32_t hal_gpio_pull(data::GpioPull pull) {
        switch (pull) {
        case data::GpioPull::kNone: return GPIO_NOPULL;
        case data::GpioPull::kUp: return GPIO_PULLUP;
        case data::GpioPull::kDown: return GPIO_PULLDOWN;
        default: core::utility::assert_failed_debug(); return GPIO_NOPULL;
        }
    }

    const ChannelHardware channel_hardware_[kChannelCount]{
        {
         .gpio_port = CHANNEL1_GPIO_Port,
         .gpio_pin = CHANNEL1_Pin,
         .alternate_function = GPIO_AF1_TIM1,
         .compare_register = &htim1.Instance->CCR1,
         },
        {
         .gpio_port = CHANNEL2_GPIO_Port,
         .gpio_pin = CHANNEL2_Pin,
         .alternate_function = GPIO_AF1_TIM1,
         .compare_register = &htim1.Instance->CCR2,
         },
        {
         .gpio_port = CHANNEL3_GPIO_Port,
         .gpio_pin = CHANNEL3_Pin,
         .alternate_function = GPIO_AF1_TIM1,
         .compare_register = &htim1.Instance->CCR3,
         },
        {
         .gpio_port = CHANNEL4_GPIO_Port,
         .gpio_pin = CHANNEL4_Pin,
         .alternate_function = GPIO_AF1_TIM1,
         .compare_register = &htim1.Instance->CCR4,
         },
        {
         .gpio_port = CHANNEL5_GPIO_Port,
         .gpio_pin = CHANNEL5_Pin,
         .alternate_function = GPIO_AF3_TIM8,
         .compare_register = &htim8.Instance->CCR1,
         },
        {
         .gpio_port = CHANNEL6_GPIO_Port,
         .gpio_pin = CHANNEL6_Pin,
         .alternate_function = GPIO_AF3_TIM8,
         .compare_register = &htim8.Instance->CCR2,
         },
        {
         .gpio_port = CHANNEL7_GPIO_Port,
         .gpio_pin = CHANNEL7_Pin,
         .alternate_function = GPIO_AF3_TIM8,
         .compare_register = &htim8.Instance->CCR3,
         },
    };

    ChannelState channel_states_[kChannelCount];
};

inline constinit Gpio::Lazy gpio;

} // namespace librmcs::firmware::gpio
