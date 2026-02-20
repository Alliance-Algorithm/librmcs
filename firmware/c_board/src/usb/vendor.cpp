#include "firmware/c_board/src/usb/vendor.hpp"

#include <cstddef>
#include <cstdint>

#include "core/src/protocol/serializer.hpp"
#include "firmware/c_board/src/usb/helper.hpp"

namespace librmcs::firmware::usb {

core::protocol::Serializer& get_serializer() { return vendor->serializer(); }

// TinyUSB device callbacks
extern "C" {

void tud_vendor_rx_cb(uint8_t itf, const uint8_t* buffer, uint16_t size) {
    if (itf != 0) [[unlikely]]
        return;

    usb::vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < Vendor::kMaxPacketSize);
}

void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }

void tud_resume_cb() {}

void tud_mount_cb() {}

void tud_umount_cb() {}

} // extern "C"

} // namespace librmcs::firmware::usb
