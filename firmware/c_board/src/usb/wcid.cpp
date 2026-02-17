// This file provides processing for WCID requests to help Windows identify the device.

// A WCID device, where WCID stands for "Windows Compatible ID", is an USB device that provides
// extra information to a Windows system, in order to facilitate automated driver installation and,
// in most circumstances, allow immediate access.

// WCID allows a device to be used by a Windows application almost as soon as it is plugged in, as
// opposed to the usual scenario where an USB device that is neither HID nor Mass Storage requires
// end-users to perform a manual driver installation. As such, WCID can bring the 'Plug-and-Play'
// functionality of HID and Mass Storage to any USB device (that sports a WCID aware firmware).

// See https://github.com/pbatard/libwdi/wiki/WCID-Devices

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <usbd_def.h>
#include <usbd_ioreq.h>

static const uint8_t kWcidStringDescriptor[] = {
    0x12,    // bLength
    0x03,    // bDescriptorType
    'M',  0, // wcChar0
    'S',  0, // wcChar1
    'F',  0, // wcChar2
    'T',  0, // wcChar3
    '1',  0, // wcChar4
    '0',  0, // wcChar5
    '0',  0, // wcChar6
    0xAE,    // bVendorCode
    0x00,    // bReserved
};

static const uint8_t kWcidFeatureDescriptor[] = {
    0x28,
    0x00,
    0x00,
    0x00, // dwLength
    0x00,
    0x01, // bcdVersion
    0x04,
    0x00, // wIndex
    0x01, // bCount
    0,
    0,
    0,
    0,
    0,
    0,
    0, // Reserved

    /* WCID Function */
    0x00, // bFirstInterfaceNumber
    0x01, // bReserved

    /* CID */
    'W',
    'I',
    'N',
    'U',
    'S',
    'B',
    0x00,
    0x00,
    /* sub CID */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,

    0,
    0,
    0,
    0,
    0,
    0, // Reserved
};

extern "C" {

bool handle_wcid_requests(USBD_HandleTypeDef* device, USBD_SetupReqTypedef* request) {
    auto send_data = [device, request]<size_t n>(const uint8_t (&data)[n]) {
        auto length = std::min<size_t>(n, request->wLength);
        // C HAL API requires non-const access.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        USBD_CtlSendData(device, const_cast<uint8_t*>(data), length);
    };

    if (request->bmRequest == (USBD_EPDirectionTypeDef::IN | USB_REQ_TYPE_STANDARD)) {
        if (request->bRequest == USB_REQ_GET_DESCRIPTOR
            && request->wValue == ((USB_DESC_TYPE_STRING << 8) | 0xEE)) {
            // Step 1: Respond with the Microsoft OS String Descriptor.
            // Indicating WCID support.
            send_data(kWcidStringDescriptor);
            return true;
        }
    } else if (request->bmRequest == (USBD_EPDirectionTypeDef::IN | USB_REQ_TYPE_VENDOR)) {
        if (request->bRequest == 0xAE && request->wIndex == 0x0004) {
            // Step 2: Respond with the Compatible ID Feature Descriptor.
            // Indicating WinUSB compatibility.
            // Note that 0xAE represents the user-defined VendorCode sent in step 1.
            send_data(kWcidFeatureDescriptor);
            return true;
        }
    }

    return false;
}

} // extern "C"
