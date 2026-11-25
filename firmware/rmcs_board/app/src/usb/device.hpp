#pragma once

namespace librmcs::firmware::usb {

class Device {
public:
    static void init(bool vendor_enabled = true);
};

} // namespace librmcs::firmware::usb
