#pragma once

#include "../utility/cross_os.hpp"
#include <cstdint>

namespace librmcs::device {

class Ch040 {
public:
private:
    PACKED_STRUCT(Quaterion {
        uint8_t label;
        float w;
        float x;
        float y;
        float z;
    };)

    PACKED_STRUCT(Package {
        uint8_t header;
        uint8_t type;
        uint16_t length;
        uint16_t crc;
        Quaterion q;
    };)
};

} // namespace librmcs::device
