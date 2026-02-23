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
#include <main.h>
#include <tusb_config.h>

#include "core/src/utility/assert.hpp"
#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class UsbDescriptors {
public:
    UsbDescriptors() { update_serial_string(); }

    static uint8_t const* get_device_descriptor() {
        return reinterpret_cast<uint8_t const*>(&kDeviceDescriptor);
    }

    static uint8_t const* get_configuration_descriptor(uint8_t index) {
        (void)index;
        return kConfigurationDescriptorFs;
    }

    uint16_t const* get_string_descriptor(uint8_t index, uint16_t langid) {
        (void)langid;
        uint8_t str_size;

        if (index == 0) {
            std::memcpy(&descriptor_string_buffer_[1], kLanguageId.data(), kLanguageId.size());
            str_size = 1;
        } else {
            std::string_view str;
            switch (index) {
            case 1: str = kManufacturerString; break;
            case 2: str = kProductString; break;
            case 3: str = std::string_view{serial_string_.data(), serial_string_.size() - 1}; break;
            default: return nullptr;
            }

            constexpr auto max_size = std::min<size_t>(
                std::tuple_size_v<decltype(descriptor_string_buffer_)> - 1,
                (std::numeric_limits<uint8_t>::max() - 2) / 2);

            str_size = static_cast<uint8_t>(std::min<size_t>(str.size(), max_size));

            for (uint8_t i = 0; i < str_size; ++i)
                descriptor_string_buffer_[i + 1] = static_cast<uint16_t>(str[i]);
        }

        descriptor_string_buffer_[0] =
            (TUSB_DESC_STRING << 8) | static_cast<uint16_t>((2 * str_size) + 2);

        return descriptor_string_buffer_.data();
    }

private:
    void update_serial_string() {
        std::array<uint32_t, 3> uid;

        uid[0] = HAL_GetUIDw0();
        uid[1] = HAL_GetUIDw1();
        uid[2] = HAL_GetUIDw2();

        mix_uid_entropy(uid);

        auto* cursor = serial_string_.data() + 3;
        for (const auto& word : uid) {
            cursor = write_hex_u16(static_cast<uint16_t>(word >> 16), cursor) + 1;
            cursor = write_hex_u16(static_cast<uint16_t>(word), cursor) + 1;
        }
        core::utility::assert_debug(cursor == serial_string_.data() + serial_string_.size());
    }

    static constexpr void mix_uid_entropy(std::array<uint32_t, 3>& uid) {
        auto& [a, b, c] = uid;

        const auto mix_step = [](uint32_t v) {
            v *= 0x9E3779B9;
            return v ^ (v >> 16);
        };

        a ^= mix_step(b ^ c);
        b ^= mix_step(a ^ c);
        c ^= mix_step(a ^ b);

        a ^= mix_step(b + c);
        b ^= mix_step(a + c);
        c ^= mix_step(a + b);

        a ^= mix_step(b ^ (c >> 5));
        b ^= mix_step(a ^ (c << 5));
        c ^= mix_step(a ^ b);

        a += mix_step(b);
        b += mix_step(c);
        c += mix_step(a);
    }

    static char* write_hex_u16(uint16_t value, char* buffer) {
        static constexpr char hex_lut[] = "0123456789ABCDEF";

        *buffer++ = hex_lut[(value >> 12) & 0xF];
        *buffer++ = hex_lut[(value >> 8) & 0xF];
        *buffer++ = hex_lut[(value >> 4) & 0xF];
        *buffer++ = hex_lut[value & 0xF];

        return buffer;
    }

private: // Device Descriptor
    static constexpr tusb_desc_device_t kDeviceDescriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,

        .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = 0xA11C,
        .idProduct = 0xD401,
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

    static constexpr uint8_t kEpnumVendorDataOut = 0x01;
    static constexpr uint8_t kEpnumVendorDataIn = 0x81;

    static constexpr uint8_t const kConfigurationDescriptorFs[] = {
        TUD_CONFIG_DESCRIPTOR(
            1, kItfNumTotal, 0, kConfigTotalLen, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
        TUD_VENDOR_DESCRIPTOR(0, 0, kEpnumVendorDataOut, kEpnumVendorDataIn, 64),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

private: // String Descriptor
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Alliance RoboMaster Team.";
    static constexpr std::string_view kProductString =
        "RMCS Agent v" LIBRMCS_PROJECT_VERSION_STRING;
    std::array<char, 33> serial_string_{"D4-0000-0000-0000-0000-0000-0000"};

    std::array<uint16_t, 128> descriptor_string_buffer_{};
};

inline constinit utility::Lazy<UsbDescriptors> usb_descriptors;

} // namespace librmcs::firmware::usb
