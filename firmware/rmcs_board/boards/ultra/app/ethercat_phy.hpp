#pragma once

#include <cstdint>

namespace librmcs::firmware::board {

class EthercatPhy {
public:
    static bool start_clock();
    static bool start_links();

private:
    static void configure_pins();
    static bool start_reference_clock();
    static bool has_phy(uint8_t address);
    static bool disable_phy1_broadcast();
    static bool configure_phy(uint8_t address);
};

} // namespace librmcs::firmware::board
