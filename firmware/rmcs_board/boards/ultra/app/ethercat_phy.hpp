#pragma once

#include <cstdint>

namespace librmcs::firmware::board {

class EthercatPhy {
public:
    static bool start_clock();
    static bool start_links();

private:
    static void configure_pins();
    static bool configure_phy(uint8_t address);
};

} // namespace librmcs::firmware::board
