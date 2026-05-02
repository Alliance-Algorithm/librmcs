#pragma once

#include <atomic>
#include <cstdint>

#include "firmware/rmcs_board/app/src/led/ws2812.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::led {

class Led {
public:
    using Lazy = utility::Lazy<Led>;

    Led() { ws2812.init(); }

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
    }

    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    void update(uint32_t tick) {
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
            if (tick & 128U)
                ws2812->set_value(255, 255, 0);
            else
                ws2812->set_value(0, 255, 255);
        } else if (uplink_full) {
            if (tick & 128U)
                ws2812->set_value(255, 255, 0);
            else
                ws2812->set_value(0, 0, 0);
        } else if (downlink_full) {
            if (tick & 128U)
                ws2812->set_value(0, 0, 0);
            else
                ws2812->set_value(0, 255, 255);
        } else {
            uint32_t brightness = (tick >> 2U) & 511U;
            if (brightness > 255U)
                brightness = 511U - brightness;
            ws2812->set_value(0, static_cast<uint8_t>(brightness), 0);
        }
    }

private:
    std::atomic<uint16_t> uplink_full_reset_counter_{0};
    std::atomic<uint16_t> downlink_full_reset_counter_{0};
};

inline constinit Led::Lazy led;

} // namespace librmcs::firmware::led
