#pragma once

#include <cstdint>

namespace librmcs::firmware::led {

class StatusLed {
public:
    static void init();
    static void set_color(uint8_t red, uint8_t green, uint8_t blue);
    static void show_assertion();
};

} // namespace librmcs::firmware::led
