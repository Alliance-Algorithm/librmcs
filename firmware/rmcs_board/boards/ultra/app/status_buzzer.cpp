#include "firmware/rmcs_board/app/src/buzzer/status_buzzer.hpp"

#include <atomic>
#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpio_regs.h>
#include <hpm_gpiom_drv.h>
#include <hpm_gpiom_regs.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_pwmv2_drv.h>
#include <hpm_soc.h>

#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::buzzer {
namespace {

class Buzzer {
public:
    Buzzer() {
        gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOF, kPin, gpiom_soc_gpio0);
        gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DO_GPIOF, kPin, 0);
        HPM_IOC->PAD[IOC_PAD_PF15].PAD_CTL = 0;
        HPM_IOC->PAD[IOC_PAD_PF15].FUNC_CTL = IOC_PF15_FUNC_CTL_GPIO_F_15;

        clock_add_to_group(clock_emds, 0);
        clock_add_to_group(clock_pwm1, 0);
        const uint32_t period = clock_get_frequency(clock_pwm1) / kToneHz;

        pwmv2_disable_counter(HPM_PWM1, kCounter);
        pwmv2_reset_counter(HPM_PWM1, kCounter);
        pwmv2_shadow_register_unlock(HPM_PWM1);
        pwmv2_set_shadow_val(HPM_PWM1, kReloadShadow, period - 1, 0, false);
        pwmv2_set_shadow_val(HPM_PWM1, kRiseShadow, 0, 0, false);
        pwmv2_set_shadow_val(HPM_PWM1, kFallShadow, period / 2, 0, false);
        pwmv2_counter_select_data_offset_from_shadow_value(HPM_PWM1, kCounter, kReloadShadow);
        pwmv2_set_reload_update_time(HPM_PWM1, kCounter, pwm_reload_update_on_shlk);
        pwmv2_counter_burst_disable(HPM_PWM1, kCounter);

        // P6 and P7 share counter 3's start logic. Even with P6 disconnected,
        // its out-of-period reset compares can prevent P7 from starting.
        pwmv2_select_cmp_source(
            HPM_PWM1, PWMV2_CMP_INDEX(12), cmp_value_from_shadow_val, kRiseShadow);
        pwmv2_select_cmp_source(
            HPM_PWM1, PWMV2_CMP_INDEX(13), cmp_value_from_shadow_val, kRiseShadow);
        pwmv2_select_cmp_source(HPM_PWM1, kRiseCompare, cmp_value_from_shadow_val, kRiseShadow);
        pwmv2_select_cmp_source(HPM_PWM1, kFallCompare, cmp_value_from_shadow_val, kFallShadow);
        pwmv2_disable_pair_mode(HPM_PWM1, pwm_channel_6);
        pwmv2_disable_four_cmp(HPM_PWM1, pwm_channel_6);

        pwmv2_config_t config{};
        config.enable_output = true;
        config.force_trigger = pwm_force_none;
        pwmv2_config_pwm(HPM_PWM1, pwm_channel_7, &config, false);
        pwmv2_shadow_register_lock(HPM_PWM1);
        pwmv2_enable_counter(HPM_PWM1, kCounter);
        pwmv2_start_pwm_output(HPM_PWM1, kCounter);
    }

    void uplink_buffer_full() { uplink_full_ticks_.store(kOverflowMs, std::memory_order::relaxed); }

    void downlink_buffer_full() {
        downlink_full_ticks_.store(kOverflowMs, std::memory_order::relaxed);
    }

    void update(uint32_t tick) {
        const bool uplink_full = consume_tick(uplink_full_ticks_);
        const bool downlink_full = consume_tick(downlink_full_ticks_);
        uint8_t beeps = 0;
        if (uplink_full && downlink_full)
            beeps = 4;
        else if (uplink_full)
            beeps = 2;
        else if (downlink_full)
            beeps = 3;
        if (beeps != beeps_) {
            beeps_ = beeps;
            pattern_start_ = tick;
        }

        if (startup_) {
            startup_ = tick <= kStartupMs;
            if (startup_) {
                set_sounding(true);
                return;
            }
            pattern_start_ = tick;
        }

        const uint32_t elapsed = tick - pattern_start_;
        if (beeps_ == 0) {
            // Keep a silent gap between the power-on chirp and the normal pattern.
            set_sounding(
                normal_prompt_pending_ && elapsed >= kNormalGapMs
                && elapsed - kNormalGapMs < kNormalToneMs);
            if (elapsed >= kNormalGapMs + kNormalToneMs)
                normal_prompt_pending_ = false;
            return;
        }

        normal_prompt_pending_ = false;
        const uint32_t phase = elapsed % kAlarmPeriodMs;
        constexpr uint32_t slot = kShortMs * 2;
        set_sounding(phase / slot < beeps_ && phase % slot < kShortMs);
    }

    [[noreturn]] void show_assertion() {
        // Interrupts are disabled. Let the watchdog reset the board while the alarm plays.
        for (;;) {
            set_sounding(true);
            clock_cpu_delay_us(kAssertionPulseUs);
            set_sounding(false);
            clock_cpu_delay_us(kAssertionPulseUs);
        }
    }

private:
    static bool consume_tick(std::atomic<uint32_t>& ticks) {
        for (auto remaining = ticks.load(std::memory_order::relaxed); remaining != 0;) {
            if (ticks.compare_exchange_weak(remaining, remaining - 1, std::memory_order::relaxed))
                return true;
        }
        return false;
    }

    void set_sounding(bool sounding) {
        if (sounding == sounding_)
            return;

        // The GPIO output latch stays low so muting always turns Q3 off.
        HPM_IOC->PAD[IOC_PAD_PF15].FUNC_CTL =
            sounding ? IOC_PF15_FUNC_CTL_PWM1_P_7 : IOC_PF15_FUNC_CTL_GPIO_F_15;
        sounding_ = sounding;
    }

    static constexpr uint8_t kPin = 15;
    static constexpr auto kCounter = pwm_counter_3;
    static constexpr uint8_t kRiseCompare = PWMV2_CMP_INDEX(14);
    static constexpr uint8_t kFallCompare = PWMV2_CMP_INDEX(15);
    static constexpr uint8_t kReloadShadow = PWMV2_SHADOW_INDEX(0);
    static constexpr uint8_t kRiseShadow = PWMV2_SHADOW_INDEX(1);
    static constexpr uint8_t kFallShadow = PWMV2_SHADOW_INDEX(2);
    static constexpr uint32_t kToneHz = 2'000;
    static constexpr uint32_t kStartupMs = 120;
    static constexpr uint32_t kShortMs = 80;
    static constexpr uint32_t kNormalToneMs = 2'000;
    static constexpr uint32_t kNormalGapMs = 1'000;
    static constexpr uint32_t kAlarmPeriodMs = 1'000;
    static constexpr uint32_t kOverflowMs = 5'000;
    static constexpr uint32_t kAssertionPulseUs = 40'000;

    std::atomic<uint32_t> uplink_full_ticks_{0};
    std::atomic<uint32_t> downlink_full_ticks_{0};
    uint32_t pattern_start_ = 0;
    uint8_t beeps_ = 0;
    bool startup_ = true;
    bool normal_prompt_pending_ = true;
    bool sounding_ = false;
};

constinit utility::Lazy<Buzzer> status_buzzer;

} // namespace

void StatusBuzzer::init() { status_buzzer.init(); }

void StatusBuzzer::update(uint32_t tick) { status_buzzer->update(tick); }

void StatusBuzzer::uplink_buffer_full() { status_buzzer->uplink_buffer_full(); }

void StatusBuzzer::downlink_buffer_full() { status_buzzer->downlink_buffer_full(); }

void StatusBuzzer::show_assertion() {
    if (auto* buzzer = status_buzzer.try_get())
        buzzer->show_assertion();
}

} // namespace librmcs::firmware::buzzer
