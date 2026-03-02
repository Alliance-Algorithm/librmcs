#include "firmware/c_board/bootloader/src/usb/usb_descriptors.hpp"

#include <cstdint>

namespace librmcs::firmware::usb {

UsbDescriptors& get_usb_descriptors() {
    static UsbDescriptors descriptors;
    return descriptors;
}

extern "C" {

uint8_t const* tud_descriptor_device_cb(void) {
    return get_usb_descriptors().get_device_descriptor();
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    return get_usb_descriptors().get_configuration_descriptor(index);
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    return get_usb_descriptors().get_string_descriptor(index, langid);
}

} // extern "C"

} // namespace librmcs::firmware::usb
