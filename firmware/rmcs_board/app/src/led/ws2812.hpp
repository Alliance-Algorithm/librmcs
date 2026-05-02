#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_dma_mgr.h>
#include <hpm_dmamux_src.h>
#include <hpm_dmav2_drv.h>
#include <hpm_dmav2_regs.h>
#include <hpm_l1c_drv.h>
#include <hpm_pwm_drv.h>
#include <hpm_pwm_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>
#include <hpm_trgm_drv.h>
#include <hpm_trgm_regs.h>
#include <hpm_trgmmux_src.h>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/gpio/analog_gpio_pin.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::led {

class Ws2812 : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Ws2812>;

    static constexpr size_t kWs2812BitCount = 24;
    static constexpr uint32_t kWs2812PwmFrequencyHz = 800'000;

    static_assert(board::kWs2812Pin.supports_pwm());
    static_assert(board::kWs2812Pin.pwm_base() == HPM_PWM1_BASE);

    explicit Ws2812() {
        board::init_ws2812_pin();

        clock_add_to_group(clock_mot0, 0);
        const uint32_t pwm_clock_hz = clock_get_frequency(clock_mot0);
        core::utility::assert_always(pwm_clock_hz >= kWs2812PwmFrequencyHz);

        const uint32_t pwm_reload = pwm_clock_hz / kWs2812PwmFrequencyHz;
        core::utility::assert_always(pwm_reload > 1);

        compare_bit0_ = static_cast<uint32_t>(static_cast<uint64_t>(pwm_reload) * 8 / 25);
        compare_bit1_ = static_cast<uint32_t>(static_cast<uint64_t>(pwm_reload) * 16 / 25);
        compare_reset_ = pwm_reload;
        compare_values_[0] = PWM_CMP_CMP_SET(compare_reset_);
        compare_values_[kWs2812BitCount + 1] = PWM_CMP_CMP_SET(compare_reset_);

        pwm_deinit(HPM_PWM1);
        pwm_set_reload(HPM_PWM1, 0, pwm_reload);
        pwm_set_start_count(HPM_PWM1, 0, 0);

        pwm_config_t pwm_config{};
        pwm_get_default_pwm_config(HPM_PWM1, &pwm_config);
        pwm_config.enable_output = true;
        pwm_config.invert_output = true;

        pwm_cmp_config_t pwm_cmp_config{};
        pwm_get_default_cmp_config(HPM_PWM1, &pwm_cmp_config);
        pwm_cmp_config.cmp = compare_reset_;
        pwm_cmp_config.update_trigger = pwm_shadow_register_update_on_modify;
        pwm_config_cmp(HPM_PWM1, board::kWs2812Pin.pwm_output_index(), &pwm_cmp_config);

        pwm_cmp_config.update_trigger = pwm_shadow_register_update_on_hw_event;
        core::utility::assert_always(
            pwm_setup_waveform(
                HPM_PWM1, board::kWs2812Pin.pwm_output_index(), &pwm_config,
                board::kWs2812Pin.pwm_output_index(), &pwm_cmp_config, 1)
            == status_success);

        pwm_cmp_config.cmp = pwm_reload - 1;
        pwm_cmp_config.update_trigger = pwm_shadow_register_update_on_modify;
        core::utility::assert_always(
            pwm_load_cmp_shadow_on_match(HPM_PWM1, 8, &pwm_cmp_config) == status_success);

        init_dma();

        trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM1_HALFRLD);
        pwm_enable_dma_request(HPM_PWM1, PWM_IRQ_HALF_RELOAD);

        pwm_start_counter(HPM_PWM1);
        pwm_issue_shadow_register_lock_event(HPM_PWM1);
    }

    bool set_value(uint8_t red, uint8_t green, uint8_t blue) {
        // The LED is too bright; reduce the brightness.
        red /= 8;
        green /= 8;
        blue /= 8;

        if (dma_channel_is_enable(dma_resource_.base, dma_resource_.channel))
            return false;

        const uint32_t grb = (static_cast<uint32_t>(green) << 16)
                           | (static_cast<uint32_t>(red) << 8) | static_cast<uint32_t>(blue);

        for (size_t i = 0; i < kWs2812BitCount; i++) {
            const bool is_one = (grb & (1UL << (kWs2812BitCount - 1 - i))) != 0;
            compare_values_[i + 1] = PWM_CMP_CMP_SET(is_one ? compare_bit1_ : compare_bit0_);
        }

        l1c_dc_writeback(
            reinterpret_cast<uintptr_t>(compare_values_.data()),
            HPM_L1C_CACHELINE_ALIGN_UP(sizeof(compare_values_)));

        auto& ctrl = dma_resource_.base->CHCTRL[dma_resource_.channel];
        ctrl.SRCADDR = reinterpret_cast<uintptr_t>(compare_values_.data());
        ctrl.TRANSIZE = compare_values_.size();
        ctrl.CTRL |= DMAV2_CHCTRL_CTRL_ENABLE_MASK;

        return true;
    }

private:
    void init_dma() {
        dma_mgr_chn_conf_t config;
        dma_mgr_get_default_chn_config(&config);

        config.en_dmamux = true;
        config.dmamux_src = HPM_DMA_SRC_MOT_0;
        config.priority = DMA_MGR_CHANNEL_PRIORITY_LOW;
        config.src_addr = reinterpret_cast<uintptr_t>(compare_values_.data());
        config.dst_addr =
            reinterpret_cast<uintptr_t>(&HPM_PWM1->CMP[board::kWs2812Pin.pwm_output_index()]);
        config.src_width = DMA_MGR_TRANSFER_WIDTH_WORD;
        config.dst_width = DMA_MGR_TRANSFER_WIDTH_WORD;
        config.src_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_INCREMENT;
        config.dst_addr_ctrl = DMA_MGR_ADDRESS_CONTROL_FIXED;
        config.src_mode = DMA_MGR_HANDSHAKE_MODE_NORMAL;
        config.dst_mode = DMA_MGR_HANDSHAKE_MODE_HANDSHAKE;
        config.src_burst_size = DMA_MGR_NUM_TRANSFER_PER_BURST_1T;
        config.size_in_byte = sizeof(compare_values_);
        config.en_infiniteloop = false;
        config.interrupt_mask = DMA_MGR_INTERRUPT_MASK_ALL;

        core::utility::assert_always(
            dma_mgr_request_specified_resource(&dma_resource_, HPM_HDMA) == status_success
            && dma_mgr_setup_channel(&dma_resource_, &config) == status_success);
    }

    dma_resource_t dma_resource_{};

    uint32_t compare_bit0_ = 0;
    uint32_t compare_bit1_ = 0;
    uint32_t compare_reset_ = 0;

    alignas(HPM_L1C_CACHELINE_SIZE) std::array<uint32_t, 1 + kWs2812BitCount + 1> compare_values_;
};

inline constinit Ws2812::Lazy ws2812;

} // namespace librmcs::firmware::led
