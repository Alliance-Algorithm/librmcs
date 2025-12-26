#include "cdc.hpp"

#include <cstddef>

namespace librmcs::firmware::usb {

// TinyUSB device callbacks
extern "C" {

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint16_t size) {
    if (itf != 0) [[unlikely]]
        return;

    std::size_t max_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
    usb::cdc->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < max_packet_size);
}

void tud_resume_cb() {}

void tud_mount_cb() {}

void tud_umount_cb() {}

} // extern "C"

} // namespace librmcs::firmware::usb
