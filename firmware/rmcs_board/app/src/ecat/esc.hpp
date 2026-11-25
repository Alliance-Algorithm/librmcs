#pragma once

#include <cstdint>

namespace librmcs::firmware::ecat {

class Esc {
public:
    static bool start();
    static void service_eeprom();
    static void read(uint16_t address, void* data, uint16_t size);
    static void write(uint16_t address, const void* data, uint16_t size);
};

} // namespace librmcs::firmware::ecat
