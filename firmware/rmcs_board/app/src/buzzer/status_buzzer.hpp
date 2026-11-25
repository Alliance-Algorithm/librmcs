#pragma once

#include <cstdint>

namespace librmcs::firmware::buzzer {

class StatusBuzzer {
public:
    static void init();
    static void update(uint32_t tick);
    static void uplink_buffer_full();
    static void downlink_buffer_full();
    static void show_assertion();
};

} // namespace librmcs::firmware::buzzer
