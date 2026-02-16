#pragma once

#include <atomic>
#include <cstdint>

#include <tim.h>

#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::led {

class Led {
public:
    Led() {
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
        reset();
    }

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order_release);
        user_controlling_.store(false, std::memory_order::relaxed);
    }

    // Lighting effect lasts for 5 seconds
    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }
    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void update(uint32_t tick) {
        // Do not change the lighting when user controlling
        if (user_controlling_.load(std::memory_order::relaxed))
            return;

        // Atomic decrease counter
        uint16_t uplink_full;
        do {
            uplink_full = uplink_full_reset_counter_.load(std::memory_order::relaxed);
            if (uplink_full == 0)
                break;
        } while (!uplink_full_reset_counter_.compare_exchange_weak(
            uplink_full, uplink_full - 1, std::memory_order::relaxed));

        uint16_t downlink_full;
        do {
            downlink_full = downlink_full_reset_counter_.load(std::memory_order::relaxed);
            if (downlink_full == 0)
                break;
        } while (!downlink_full_reset_counter_.compare_exchange_weak(
            downlink_full, downlink_full - 1, std::memory_order::relaxed));

        if (uplink_full && downlink_full) {
            // Both full: yellow and aqua lights flashing alternately
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 255, 255);
        } else if (uplink_full) {
            // Uplink full: yellow light flashing
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 0, 0);
        } else if (downlink_full) {
            // Downlink full: aqua light flashing
            if (tick & 128)
                set_value(0, 0, 0);
            else
                set_value(0, 255, 255);
        } else {
            // Working well: green breathing light
            auto brightness = (tick >> 2) & 511;
            if (brightness > 255)
                brightness = 511 - brightness;
            set_value(0, brightness, 0);
        }
    }

    // Non-static to ensure instantiation
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void set_value(uint8_t red, uint8_t green, uint8_t blue) {
        htim5.Instance->CCR1 = blue;
        htim5.Instance->CCR2 = green;
        htim5.Instance->CCR3 = red;
    }

private:
    std::atomic<bool> user_controlling_;

    std::atomic<uint16_t> uplink_full_reset_counter_, downlink_full_reset_counter_;
};

inline utility::Lazy<Led> led;

} // namespace librmcs::firmware::led
