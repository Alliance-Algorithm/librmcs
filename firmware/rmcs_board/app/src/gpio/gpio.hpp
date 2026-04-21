#pragma once

#include <cstddef>
#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpio_regs.h>
#include <hpm_ioc_regs.h>
#include <hpm_mchtmr_drv.h>
#include <hpm_pwm_drv.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "board_app.hpp"
#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/gpio/analog_gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/usb/helper.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::gpio {

class Gpio : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Gpio>;

    Gpio()
        : mchtmr_frequency_hz_(clock_get_frequency(clock_mchtmr0)) {
        board::init_gpio_pins();
    }

    void handle_digital_write(uint8_t channel_index, const data::GpioDigitalDataView& data) {
        configure_digital_output_mode(channel_index, data.high);
    }

    void handle_analog_write(uint8_t channel_index, const data::GpioAnalogDataView& data) {
        configure_pwm_output_mode(channel_index);
        channel_hardware(channel_index)
            .update_pwm_compare_edge_aligned(duty16_to_pwm_compare(data.value));
    }

    void handle_digital_read(uint8_t channel_index, const data::GpioReadConfigView& data) {
        configure_digital_input_mode(channel_index, data);

        if (data.asap)
            publish_digital_input_sample(channel_index);
    }

    void poll_periodic_input_samples() {
        const uint64_t now = now_ticks();

        for (std::size_t channel_index = 0; channel_index < board::spec::kGpioDescriptors.size();
             ++channel_index) {
            auto& state = channel_states_[channel_index];
            if (state.mode != ChannelMode::kDigitalInput || state.period_ticks == 0)
                continue;
            if (now < state.next_sample_tick)
                continue;

            publish_digital_input_sample(static_cast<uint8_t>(channel_index));
            state.next_sample_tick = now + state.period_ticks;
        }
    }

    void handle_port_interrupt(uint32_t port_index) {
        for (std::size_t channel_index = 0; channel_index < board::spec::kGpioDescriptors.size();
             ++channel_index) {
            const auto& hardware = channel_hardware(static_cast<uint8_t>(channel_index));
            if (hardware.port() != port_index)
                continue;
            if (!hardware.check_clear_interrupt_flag())
                continue;

            auto& state = channel_states_[channel_index];
            if (state.mode != ChannelMode::kDigitalInput
                || (!state.rising_edge && !state.falling_edge))
                continue;

            publish_digital_input_sample(static_cast<uint8_t>(channel_index));
        }
    }

private:
    enum class ChannelMode : uint8_t {
        kUnconfigured = 0,
        kDigitalOutput = 1,
        kDigitalInput = 2,
        kPwmOutput = 3,
    };

    struct ChannelState {
        ChannelMode mode = ChannelMode::kUnconfigured;
        bool rising_edge = false;
        bool falling_edge = false;
        data::GpioPull pull = data::GpioPull::kNone;
        uint64_t period_ticks = 0;
        uint64_t next_sample_tick = 0;
    };
    static constexpr uint32_t kPwmFrequencyHz = 50;

    void configure_digital_output_mode(uint8_t channel_index, bool initial_high) {
        auto& state = channel_state(channel_index);
        const auto& hardware = channel_hardware(channel_index);

        if (state.mode != ChannelMode::kDigitalOutput) {
            prepare_pin_for_digital_output(hardware, initial_high);
            set_output_mode_state(state, ChannelMode::kDigitalOutput);
            return;
        }

        hardware.write_pin(initial_high);
    }

    ChannelState& channel_state(uint8_t channel_index) {
        core::utility::assert_debug(channel_index < board::spec::kGpioDescriptors.size());
        return channel_states_[channel_index];
    }

    static const AnalogGpioPin& channel_hardware(uint8_t channel_index) {
        core::utility::assert_debug(channel_index < board::spec::kGpioDescriptors.size());
        return board::kGpioHardwareDescriptors[channel_index];
    }

    static void prepare_pin_for_digital_output(const AnalogGpioPin& hardware, bool initial_high) {
        hardware.disable_interrupt();
        hardware.configure_as_gpio();
        hardware.configure_pad_control(0);
        hardware.write_pin(initial_high);
        hardware.configure_as_output();
    }

    void prepare_pin_for_pwm_output(const AnalogGpioPin& hardware) {
        hardware.disable_interrupt();
        ensure_pwm_initialized();
        hardware.configure_pad_control(0);
        hardware.configure_as_pwm();
    }

    static void set_output_mode_state(ChannelState& state, ChannelMode mode) {
        state.mode = mode;
        state.rising_edge = false;
        state.falling_edge = false;
        state.pull = data::GpioPull::kNone;
        state.period_ticks = 0;
        state.next_sample_tick = 0;
    }

    void configure_pwm_output_mode(uint8_t channel_index) {
        auto& state = channel_state(channel_index);
        const auto& hardware = channel_hardware(channel_index);

        core::utility::assert_debug(hardware.supports_pwm());
        if (state.mode == ChannelMode::kPwmOutput)
            return;

        prepare_pin_for_pwm_output(hardware);
        set_output_mode_state(state, ChannelMode::kPwmOutput);
    }

    static void set_digital_input_mode_state(
        ChannelState& state, const data::GpioReadConfigView& data, uint64_t period_ticks) {
        state.mode = ChannelMode::kDigitalInput;
        state.rising_edge = data.rising_edge;
        state.falling_edge = data.falling_edge;
        state.pull = data.pull;
        state.period_ticks = period_ticks;
        state.next_sample_tick = now_ticks();
    }

    void ensure_pwm_initialized() {
        if (pwm_initialized_)
            return;

        clock_add_to_group(clock_mot0, 0);
        const uint32_t pwm_clock_hz = clock_get_frequency(clock_mot0);
        core::utility::assert_always(pwm_clock_hz >= kPwmFrequencyHz);

        pwm_reload_ = pwm_clock_hz / kPwmFrequencyHz;
        core::utility::assert_always(pwm_reload_ > 1);

        pwm_deinit(HPM_PWM0);
        pwm_set_reload(HPM_PWM0, 0, pwm_reload_);
        pwm_set_start_count(HPM_PWM0, 0, 0);

        pwm_config_t pwm_config{};
        pwm_get_default_pwm_config(HPM_PWM0, &pwm_config);
        pwm_config.enable_output = true;
        pwm_config.invert_output = false;

        for (const auto& hardware : board::kGpioHardwareDescriptors) {
            if (!hardware.supports_pwm())
                continue;

            core::utility::assert_always(
                hardware.setup_pwm_waveform_edge_aligned(pwm_reload_, pwm_config)
                == status_success);
        }

        pwm_start_counter(HPM_PWM0);
        pwm_issue_shadow_register_lock_event(HPM_PWM0);
        pwm_initialized_ = true;
    }

    [[nodiscard]] uint32_t duty16_to_pwm_compare(uint16_t duty) const {
        if (duty == 0)
            return pwm_reload_ + 1;
        return (static_cast<uint64_t>(pwm_reload_) * (65535U - duty)) / 65535U;
    }

    void configure_digital_input_mode(uint8_t channel_index, const data::GpioReadConfigView& data) {
        auto& state = channel_state(channel_index);
        const uint64_t period_ticks = period_ms_to_ticks(data.period_ms);
        if (!needs_digital_input_reconfigure(state, data, period_ticks))
            return;

        const auto& hardware = channel_hardware(channel_index);
        prepare_pin_for_digital_input(hardware, data.pull);
        set_digital_input_mode_state(state, data, period_ticks);

        if (data.rising_edge || data.falling_edge)
            configure_interrupt(hardware, data.rising_edge, data.falling_edge);
    }

    [[nodiscard]] uint64_t period_ms_to_ticks(uint16_t period_ms) const {
        if (period_ms == 0)
            return 0;
        return (static_cast<uint64_t>(mchtmr_frequency_hz_) * period_ms) / 1000U;
    }

    static bool needs_digital_input_reconfigure(
        const ChannelState& state, const data::GpioReadConfigView& data, uint64_t period_ticks) {
        return state.mode != ChannelMode::kDigitalInput || state.rising_edge != data.rising_edge
            || state.falling_edge != data.falling_edge || state.pull != data.pull
            || state.period_ticks != period_ticks;
    }

    static uint32_t pad_control_from_pull(data::GpioPull pull) {
        switch (pull) {
        case data::GpioPull::kNone: return IOC_PAD_PAD_CTL_HYS_SET(1);
        case data::GpioPull::kUp:
            return IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1)
                 | IOC_PAD_PAD_CTL_HYS_SET(1);
        case data::GpioPull::kDown:
            return IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0)
                 | IOC_PAD_PAD_CTL_HYS_SET(1);
        default: core::utility::assert_failed_debug(); return IOC_PAD_PAD_CTL_HYS_SET(1);
        }
    }

    static void prepare_pin_for_digital_input(const AnalogGpioPin& hardware, data::GpioPull pull) {
        hardware.disable_interrupt();
        hardware.configure_as_gpio();
        hardware.configure_pad_control(pad_control_from_pull(pull));
        hardware.configure_as_input();
    }

    [[nodiscard]] static uint64_t now_ticks() { return mchtmr_get_count(HPM_MCHTMR); }

    static void
        configure_interrupt(const AnalogGpioPin& hardware, bool rising_edge, bool falling_edge) {
        gpio_interrupt_trigger_t trigger = gpio_interrupt_trigger_edge_rising;
        if (rising_edge && falling_edge) {
            trigger = gpio_interrupt_trigger_edge_both;
        } else if (falling_edge) {
            trigger = gpio_interrupt_trigger_edge_falling;
        }

        hardware.configure_interrupt(trigger);
        hardware.clear_interrupt_flag();
        hardware.enable_interrupt();
        enable_port_irq(hardware.port());
    }

    static void enable_port_irq(uint32_t port_index) {
        switch (port_index) {
        case GPIO_DI_GPIOA: intc_m_enable_irq_with_priority(IRQn_GPIO0_A, 1); break;
        case GPIO_DI_GPIOB: intc_m_enable_irq_with_priority(IRQn_GPIO0_B, 1); break;
        case GPIO_DI_GPIOY: intc_m_enable_irq_with_priority(IRQn_GPIO0_Y, 1); break;
        default: core::utility::assert_failed_debug();
        }
    }

    static void publish_digital_input_sample(uint8_t channel_index) {
        const auto& hardware = channel_hardware(channel_index);
        const bool high = hardware.read_pin();

        auto& serializer = usb::get_serializer();
        core::utility::assert_debug(
            serializer.write_gpio_digital_read_result(channel_index, {.high = high})
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    const uint32_t mchtmr_frequency_hz_;
    uint32_t pwm_reload_ = 0;
    bool pwm_initialized_ = false;
    ChannelState channel_states_[board::spec::kGpioDescriptors.size()]{};
};

inline constinit Gpio::Lazy gpio;

} // namespace librmcs::firmware::gpio
