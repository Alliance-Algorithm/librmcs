#pragma once

#include <common/tusb_types.h>

namespace librmcs::firmware::usb {

class Device {
public:
    static void init(tusb_speed_t speed, bool vendor_enabled = true);
};

} // namespace librmcs::firmware::usb
