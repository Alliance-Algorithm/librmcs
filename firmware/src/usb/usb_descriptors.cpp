#include "firmware/src/usb/usb_descriptors.hpp"

#include <cstdint>

namespace librmcs::firmware::usb {

// TinyUSB descriptor callbacks
extern "C" {

// Device Descriptors
// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const* tud_descriptor_device_cb(void) { return usb_descriptors->get_device_descriptor(); }

// Configuration Descriptors
// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    return usb_descriptors->get_configuration_descriptor(index);
}

// String Descriptors
// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    return usb_descriptors->get_string_descriptor(index, langid);
}

} // extern "C"

} // namespace librmcs::firmware::usb
