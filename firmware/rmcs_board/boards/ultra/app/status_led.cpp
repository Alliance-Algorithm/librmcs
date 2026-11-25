#include "firmware/rmcs_board/app/src/led/status_led.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <board.h>
#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_dma_mgr.h>
#include <hpm_dmamux_src.h>
#include <hpm_dmav2_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_pwmv2_drv.h>
#include <hpm_pwmv2_regs.h>
#include <hpm_soc.h>
#include <hpm_trgm_drv.h>
#include <hpm_trgm_regs.h>
#include <hpm_trgmmux_src.h>
#include <riscv/riscv_core.h>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::led {
namespace {

class Ws2812PwmV2 : private core::utility::Immovable {
public:
    Ws2812PwmV2() {
        // TRGM shares the motor subsystem clock on HPM5E31.
        clock_add_to_group(clock_emds, 0);
        clock_add_to_group(clock_pwm0, 0);
        const uint32_t pwm_clock_hz = clock_get_frequency(clock_pwm0);
        const uint32_t period_ticks = (pwm_clock_hz + (kBitRateHz / 2)) / kBitRateHz;
        core::utility::assert_always(period_ticks >= kDutyDenominator);

        init_pwm(period_ticks);
        init_dma();

        trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM0_2);
        pwmv2_enable_dma_at_reload_point(HPM_PWM0, kDmaTrigger, kCounter);
        pwmv2_enable_counter(HPM_PWM0, kCounter);
        pwmv2_start_pwm_output(HPM_PWM0, kCounter);
        HPM_IOC->PAD[IOC_PAD_PE05].FUNC_CTL = IOC_PE05_FUNC_CTL_PWM0_P_5;
        // Start the initial low interval once the pin is driven.
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        last_send_tick_ = timer::Timer::timestamp_quarter_us();
    }

    void set_value(uint8_t red, uint8_t green, uint8_t blue) {
        const uint32_t now = timer::Timer::timestamp_quarter_us();
        if (now - last_send_tick_ < kMinFrameSpacingTicks
            || dma_channel_is_enable(dma_resource_.base, dma_resource_.channel))
            return;

        const uint32_t grb = (static_cast<uint32_t>(green / kBrightnessDivisor) << 16)
                           | (static_cast<uint32_t>(red / kBrightnessDivisor) << 8)
                           | static_cast<uint32_t>(blue / kBrightnessDivisor);
        // Start from idle even if the DMA channel is armed between reloads.
        frame_.front() = compare_config(kIdleShadow);
        for (size_t i = 0; i < kBitCount; ++i) {
            const bool one = ((grb >> (kBitCount - 1 - i)) & 1U) != 0;
            frame_[i + 1] = compare_config(one ? kOneShadow : kZeroShadow);
        }
        // Leave the pin low after the final bit until the next frame.
        frame_.back() = compare_config(kIdleShadow);

        dma_clear_transfer_status(dma_resource_.base, dma_resource_.channel);
        dma_set_source_address(
            dma_resource_.base, dma_resource_.channel, reinterpret_cast<uintptr_t>(frame_.data()));
        dma_set_transfer_size(dma_resource_.base, dma_resource_.channel, frame_.size());
        // Publish the frame and DMA registers before enabling the channel.
        std::atomic_signal_fence(std::memory_order_release);
        fenceiorw();
        (void)dma_enable_channel(dma_resource_.base, dma_resource_.channel);
        last_send_tick_ = timer::Timer::timestamp_quarter_us();
    }

private:
    static constexpr size_t kBitCount = 24;
    static constexpr uint32_t kBitRateHz = 800'000;
    static constexpr uint8_t kBrightnessDivisor = 8;
    static constexpr uint32_t kDutyDenominator = 25;
    static constexpr uint32_t kZeroDutyNumerator = 8;
    static constexpr uint32_t kOneDutyNumerator = 16;
    static constexpr uint32_t kRiseTick = 1;
    // Allow for the ~33 us transfer and >280 us latch interval even if two
    // status ticks are serviced close together after an interrupt delay.
    static constexpr uint32_t kMinFrameSpacingUs = 350;
    static constexpr uint32_t kMinFrameSpacingTicks =
        kMinFrameSpacingUs * (timer::Timer::kTimerFrequencyHz / 1'000'000U);

    // PWM0.P5 uses counter 2 and compare units 10 and 11 in independent mode.
    static constexpr auto kCounter = pwm_counter_2;
    static constexpr auto kOutput = pwm_channel_5;
    // Use the same-index DMA output for the counter's reload event.
    static constexpr auto kDmaTrigger = pwm_dma_2;
    static constexpr uint8_t kPwm4RiseCompare = PWMV2_CMP_INDEX(8);
    static constexpr uint8_t kPwm4FallCompare = PWMV2_CMP_INDEX(9);
    static constexpr uint8_t kRiseCompare = PWMV2_CMP_INDEX(10);
    static constexpr uint8_t kFallCompare = PWMV2_CMP_INDEX(11);
    static constexpr uint8_t kReloadShadow = PWMV2_SHADOW_INDEX(0);
    static constexpr uint8_t kRiseShadow = PWMV2_SHADOW_INDEX(1);
    static constexpr uint8_t kIdleShadow = PWMV2_SHADOW_INDEX(2);
    static constexpr uint8_t kZeroShadow = PWMV2_SHADOW_INDEX(3);
    static constexpr uint8_t kOneShadow = PWMV2_SHADOW_INDEX(4);

    using Frame = std::array<uint32_t, kBitCount + 2>;
    static Frame frame_;

    static constexpr uint32_t compare_config(uint8_t shadow) {
        return PWMV2_CMP_CFG_CMP_IN_SEL_SET(shadow)
             | PWMV2_CMP_CFG_CMP_UPDATE_TIME_SET(pwm_shadow_register_update_on_reload);
    }

    static void init_pwm(uint32_t period_ticks) {
        pwmv2_disable_counter(HPM_PWM0, kCounter);
        pwmv2_reset_counter(HPM_PWM0, kCounter);
        pwmv2_disable_dma_at_reload_point(HPM_PWM0, kDmaTrigger);

        pwmv2_enable_shadow_lock_feature(HPM_PWM0);
        pwmv2_shadow_register_unlock(HPM_PWM0);
        pwmv2_set_shadow_val(HPM_PWM0, kReloadShadow, period_ticks - 1, 0, false);
        pwmv2_set_shadow_val(HPM_PWM0, kRiseShadow, kRiseTick, 0, false);
        // A falling compare below the rising compare keeps the output low.
        pwmv2_set_shadow_val(HPM_PWM0, kIdleShadow, 0, 0, false);
        pwmv2_set_shadow_val(
            HPM_PWM0, kZeroShadow,
            kRiseTick + (period_ticks * kZeroDutyNumerator / kDutyDenominator), 0, false);
        pwmv2_set_shadow_val(
            HPM_PWM0, kOneShadow, kRiseTick + (period_ticks * kOneDutyNumerator / kDutyDenominator),
            0, false);

        pwmv2_counter_select_data_offset_from_shadow_value(HPM_PWM0, kCounter, kReloadShadow);
        pwmv2_set_reload_update_time(HPM_PWM0, kCounter, pwm_reload_update_on_shlk);
        pwmv2_counter_burst_disable(HPM_PWM0, kCounter);

        // On HPM5E31, P4's out-of-range falling compare at reset can hold P5 low.
        // Give P4 an idle compare window even though its output is disabled.
        HPM_PWM0->CMP[kPwm4RiseCompare].CFG = PWMV2_CMP_CFG_CMP_IN_SEL_SET(kRiseShadow);
        HPM_PWM0->CMP[kPwm4FallCompare].CFG = PWMV2_CMP_CFG_CMP_IN_SEL_SET(kIdleShadow);
        HPM_PWM0->CMP[kRiseCompare].CFG = PWMV2_CMP_CFG_CMP_IN_SEL_SET(kRiseShadow);
        HPM_PWM0->CMP[kFallCompare].CFG = PWMV2_CMP_CFG_CMP_IN_SEL_SET(kIdleShadow);
        pwmv2_disable_pair_mode(HPM_PWM0, pwm_channel_4);
        pwmv2_disable_four_cmp(HPM_PWM0, pwm_channel_4);
        pwmv2_config_t config{};
        config.enable_output = true;
        config.force_trigger = pwm_force_none;
        pwmv2_config_pwm(HPM_PWM0, kOutput, &config, false);
        pwmv2_shadow_register_lock(HPM_PWM0);

        // Each reload latches the selected falling edge and requests the DMA
        // write for the next period. DMA selects a preloaded value without
        // rewriting the locked shadow registers or changing the current pulse.
        HPM_PWM0->CMP[kFallCompare].CFG = compare_config(kIdleShadow);
    }

    void init_dma() {
        dma_mgr_chn_conf_t config;
        dma_mgr_get_default_chn_config(&config);
        config.en_dmamux = true;
        config.dmamux_src = HPM_DMA_SRC_MOT_0;
        config.src_addr = reinterpret_cast<uintptr_t>(frame_.data());
        config.dst_addr = reinterpret_cast<uintptr_t>(&HPM_PWM0->CMP[kFallCompare].CFG);
        config.src_width = DMA_MGR_TRANSFER_WIDTH_WORD;
        config.dst_width = DMA_MGR_TRANSFER_WIDTH_WORD;
        config.dst_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_FIXED;
        config.dst_mode = DMA_MGR_HANDSHAKE_MODE_HANDSHAKE;
        config.size_in_byte = sizeof(frame_);

        core::utility::assert_always(
            dma_mgr_request_specified_resource(&dma_resource_, HPM_HDMA) == status_success
            && dma_mgr_setup_channel(&dma_resource_, &config) == status_success);
    }

    dma_resource_t dma_resource_{};
    uint32_t last_send_tick_ = 0;
};

ATTR_PLACE_AT_NONCACHEABLE_BSS constinit Ws2812PwmV2::Frame Ws2812PwmV2::frame_{};

constinit utility::Lazy<Ws2812PwmV2> ws2812;

} // namespace

void StatusLed::init() { ws2812.init(); }

void StatusLed::set_color(uint8_t red, uint8_t green, uint8_t blue) {
    ws2812->set_value(red, green, blue);
}

void StatusLed::show_assertion() {
    if (auto* driver = ws2812.try_get()) {
        for (int i = 0; i < 10; ++i)
            board_delay_ms(10);
        driver->set_value(255, 0, 0);
    }
}

} // namespace librmcs::firmware::led
