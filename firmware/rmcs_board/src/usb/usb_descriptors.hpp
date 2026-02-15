#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <tuple>

#include <common/tusb_types.h>
#include <device/usbd.h>
#include <tusb_config.h>
#include <tusb_option.h>

#include "firmware/rmcs_board/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class UsbDescriptors {
public:
    uint8_t const* get_device_descriptor() const {
        return reinterpret_cast<uint8_t const*>(&device_descriptor_);
    }

    static uint8_t const* get_configuration_descriptor(uint8_t index) {
        (void)index; // For multiple configurations

        if constexpr (TUD_OPT_HIGH_SPEED)
            return (tud_speed_get() == TUSB_SPEED_HIGH) ? kConfigurationDescriptorHs
                                                        : kConfigurationDescriptorFs;
        else
            return kConfigurationDescriptorFs;
    }

    uint16_t const* get_string_descriptor(uint8_t index, uint16_t langid) {
        (void)langid;
        uint8_t str_size;

        if (index == 0) {
            std::memcpy(&descriptor_string_buffer_[1], kStringDescriptor[0].data(), 2);
            str_size = 1;
        } else {
            // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
            // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

            constexpr auto string_descriptor_array_size =
                sizeof(kStringDescriptor) / sizeof(kStringDescriptor[0]);
            if (index >= string_descriptor_array_size)
                return nullptr;

            constexpr auto max_size = std::min<size_t>(
                std::tuple_size_v<decltype(descriptor_string_buffer_)> - 1,
                (std::numeric_limits<uint8_t>::max() - 2) / 2);

            const auto& str = kStringDescriptor[index];
            str_size = std::min<size_t>(str.size(), max_size);

            // Convert ASCII string into UTF-16
            for (uint8_t i = 0; i < str_size; i++)
                descriptor_string_buffer_[i + 1] = str[i];
        }

        // first byte is length (including header), second byte is string type
        descriptor_string_buffer_[0] = (TUSB_DESC_STRING << 8) | ((2 * str_size) + 2);

        return descriptor_string_buffer_.data();
    }

private: // Device Descriptor
    tusb_desc_device_t const device_descriptor_ = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,

        .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = 0xa11c,
        .idProduct = 0x1235, // TODO: Generate dynamically
        .bcdDevice = 0x0100,

        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,

        .bNumConfigurations = 0x01,
    };

private: // Configuration Descriptor
    static constexpr size_t kItfNumTotal = 1;

    static constexpr size_t kConfigTotalLen =
        TUD_CONFIG_DESC_LEN + CFG_TUD_VENDOR * TUD_VENDOR_DESC_LEN;

    // Align endpoint numbering to STM32 HAL style:
    // EP1 OUT: data OUT, EP1 IN: data IN
    static constexpr uint8_t kEpnumCdc0DataOut = 0x01;
    static constexpr uint8_t kEpnumCdc0DataIn = 0x81;

    static constexpr uint8_t const kConfigurationDescriptorFs[] = {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(
            1, kItfNumTotal, 0, kConfigTotalLen, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

        // Interface number, string index, EP data address (out, in) and size.
        TUD_VENDOR_DESCRIPTOR(0, 0, kEpnumCdc0DataOut, kEpnumCdc0DataIn, 64),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

    static constexpr uint8_t const kConfigurationDescriptorHs[] = {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(
            1, kItfNumTotal, 0, kConfigTotalLen, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

        // Interface number, string index, EP data address (out, in) and size.
        TUD_VENDOR_DESCRIPTOR(0, 0, kEpnumCdc0DataOut, kEpnumCdc0DataIn, 512),
    };
    static_assert(sizeof(kConfigurationDescriptorHs) == kConfigTotalLen);

private:                                               // String Descriptor
    static constexpr std::string_view kStringDescriptor[4] = {
        "\x09\x04",                                    // 0: Support English (0x0409)
        "Alliance RoboMaster Team.",                   // 1: Manufacturer
        "RMCS Board v" LIBRMCS_PROJECT_VERSION_STRING, // 2: Product
        "123456",                                      // 3: Serials, should use chip ID
    };
    std::array<uint16_t, 128> descriptor_string_buffer_;
};
inline constinit utility::Lazy<UsbDescriptors> usb_descriptors;

} // namespace librmcs::firmware::usb
