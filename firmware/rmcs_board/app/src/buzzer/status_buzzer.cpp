#include "firmware/rmcs_board/app/src/buzzer/status_buzzer.hpp"

#include <cstdint>

namespace librmcs::firmware::buzzer {

void StatusBuzzer::init() {}

void StatusBuzzer::update(uint32_t) {}

void StatusBuzzer::uplink_buffer_full() {}

void StatusBuzzer::downlink_buffer_full() {}

void StatusBuzzer::show_assertion() {}

} // namespace librmcs::firmware::buzzer
