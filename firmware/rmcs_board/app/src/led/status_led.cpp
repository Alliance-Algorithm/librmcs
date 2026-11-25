#include "firmware/rmcs_board/app/src/led/status_led.hpp"

#include <cstdint>

#include <board.h>

#include "firmware/rmcs_board/app/src/led/ws2812.hpp"

namespace librmcs::firmware::led {

void StatusLed::init() { ws2812.init(); }

void StatusLed::set_color(uint8_t red, uint8_t green, uint8_t blue) {
    ws2812->set_value(red, green, blue);
}

void StatusLed::show_assertion() {
    if (auto* driver = ws2812.try_get()) {
        for (int i = 0; i < 10; i++)
            board_delay_ms(10);
        driver->set_value(255, 0, 0);
    }
}

} // namespace librmcs::firmware::led
