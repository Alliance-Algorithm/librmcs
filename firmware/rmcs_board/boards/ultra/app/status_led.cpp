#include "firmware/rmcs_board/app/src/led/status_led.hpp"

#include <cstdint>

namespace librmcs::firmware::led {

void StatusLed::init() {}

void StatusLed::set_color(uint8_t, uint8_t, uint8_t) {}

void StatusLed::show_assertion() {}

} // namespace librmcs::firmware::led
