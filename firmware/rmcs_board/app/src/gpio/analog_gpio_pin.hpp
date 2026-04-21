#pragma once

#include <cstdint>

#include <hpm_common.h> // IWYU pragma: keep
#include <hpm_gpio_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_pwm_drv.h>
#include <hpm_pwm_regs.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"

namespace librmcs::firmware {

class AnalogGpioPin : public GpioPin {
public:
    constexpr AnalogGpioPin(const GpioPin& pin) // NOLINT(google-explicit-constructor)
        : GpioPin(pin)
        , pwm_base_(0)
        , pwm_ioc_function_(0)
        , pwm_output_index_(0) {}

    constexpr AnalogGpioPin(
        const GpioPin& pin, uintptr_t pwm_base, uint32_t pwm_ioc_function, uint8_t pwm_output_index)
        : GpioPin(pin)
        , pwm_base_(pwm_base)
        , pwm_ioc_function_(static_cast<uint8_t>(pwm_ioc_function))
        , pwm_output_index_(pwm_output_index) {
        core::utility::assert_debug(pwm_ioc_function < 32 && pwm_output_index < 8);
    }

    [[nodiscard]] constexpr bool supports_pwm() const noexcept { return pwm_base_ != 0; }

    void configure_as_gpio() const {
        configure_controller();
        configure_ioc_function();
    }

    void configure_as_pwm() const {
        core::utility::assert_debug(supports_pwm());
        configure_controller();
        configure_ioc_function(pwm_ioc_function_);
    }

    void disable_interrupt() const {
        gpio_disable_pin_interrupt(gpio_instance(), port(), pin());
        clear_interrupt_flag();
    }

    void update_pwm_compare_edge_aligned(uint32_t compare_value) const {
        core::utility::assert_debug(supports_pwm());
        pwm_update_raw_cmp_edge_aligned(pwm_instance(), pwm_output_index_, compare_value);
    }

    hpm_stat_t setup_pwm_waveform_edge_aligned(uint32_t reload, pwm_config_t& pwm_config) const {
        core::utility::assert_debug(supports_pwm());

        pwm_cmp_config_t cmp_config{};
        pwm_get_default_cmp_config(pwm_instance(), &cmp_config);
        cmp_config.cmp = reload + 1;

        return pwm_setup_waveform(
            pwm_instance(), pwm_output_index_, &pwm_config, pwm_output_index_, &cmp_config, 1);
    }

private:
    PWM_Type* pwm_instance() const { return reinterpret_cast<PWM_Type*>(pwm_base_); }

    uintptr_t pwm_base_;

    uint8_t pwm_ioc_function_;
    uint8_t pwm_output_index_;

    static_assert(IOC_PAD_FUNC_CTL_ALT_SELECT_SET(31) == 31);
};

} // namespace librmcs::firmware
