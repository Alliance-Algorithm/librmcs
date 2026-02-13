#include "firmware/rmcs_board/src/usb/vendor.hpp"

#include <cstddef>
#include <cstdint>

#include <common/tusb_types.h>
#include <device/usbd.h>

#include "core/src/protocol/serializer.hpp"

namespace librmcs::firmware::usb {

core::protocol::Serializer& get_serializer() { return vendor->serializer(); }

// TinyUSB device callbacks
extern "C" {

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint16_t size) {
    if (itf != 0) [[unlikely]]
        return;

    const std::size_t max_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < max_packet_size);
}

void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }

void tud_resume_cb() {}

void tud_mount_cb() {}

void tud_umount_cb() {}

} // extern "C"

} // namespace librmcs::firmware::usb
