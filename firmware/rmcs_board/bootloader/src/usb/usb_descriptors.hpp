#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <tuple>

#include <class/dfu/dfu.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <hpm_otp_drv.h>
#include <hpm_soc_feature.h>
#include <tusb_config.h>

#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"

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
        uint8_t str_size = 0U;

        if (index == 0U) {
            std::memcpy(&descriptor_string_buffer_[1], kLanguageId.data(), kLanguageId.size());
            str_size = 1U;
        } else {
            std::string_view str;
            switch (index) {
            case 1: str = kManufacturerString; break;
            case 2: str = kProductString; break;
            case 3:
                str = std::string_view{serial_string_.data(), serial_string_.size() - 1U};
                break;
            case 4: str = kAlt0String; break;
            default: return nullptr;
            }

            constexpr auto max_size = std::min<size_t>(
                std::tuple_size_v<decltype(descriptor_string_buffer_)> - 1U,
                (std::numeric_limits<uint8_t>::max() - 2U) / 2U);
            str_size = static_cast<uint8_t>(std::min<size_t>(str.size(), max_size));

            for (uint8_t i = 0U; i < str_size; ++i)
                descriptor_string_buffer_[i + 1U] = static_cast<uint16_t>(str[i]);
        }

        descriptor_string_buffer_[0] =
            (TUSB_DESC_STRING << 8) | static_cast<uint16_t>((2U * str_size) + 2U);
        return descriptor_string_buffer_.data();
    }

private:
    static constexpr size_t kUuidWordCount = OTP_SOC_UUID_LEN / sizeof(uint32_t);
    static_assert((OTP_SOC_UUID_LEN % sizeof(uint32_t)) == 0U);

    void update_serial_string() {
        std::array<uint32_t, kUuidWordCount> uuid{};

        for (size_t i = 0U; i < uuid.size(); ++i)
            uuid[i] = otp_read_from_shadow(OTP_SOC_UUID_IDX + static_cast<uint32_t>(i));

        mix_uid_entropy(uuid);

        auto* cursor = serial_string_.data() + 3;
        for (const auto& word : uuid) {
            cursor = write_hex_u16(static_cast<uint16_t>(word >> 16U), cursor) + 1;
            cursor = write_hex_u16(static_cast<uint16_t>(word), cursor) + 1;
        }
        utility::assert_debug(cursor == serial_string_.data() + serial_string_.size());
    }

    static constexpr void mix_uid_entropy(std::array<uint32_t, kUuidWordCount>& uid) {
        static_assert(kUuidWordCount == 4U);

        auto& [a, b, c, d] = uid;

        const auto mix_step = [](uint32_t v) {
            v *= 0x9E3779B9U;
            return v ^ (v >> 16U);
        };

        a ^= mix_step(b ^ c ^ d);
        b ^= mix_step(a ^ c ^ d);
        c ^= mix_step(a ^ b ^ d);
        d ^= mix_step(a ^ b ^ c);

        a ^= mix_step(b + c + d);
        b ^= mix_step(a + c + d);
        c ^= mix_step(a + b + d);
        d ^= mix_step(a + b + c);
    }

    static char* write_hex_u16(uint16_t value, char* buffer) {
        static constexpr char hex_lut[] = "0123456789ABCDEF";

        *buffer++ = hex_lut[(value >> 12U) & 0xFU];
        *buffer++ = hex_lut[(value >> 8U) & 0xFU];
        *buffer++ = hex_lut[(value >> 4U) & 0xFU];
        *buffer++ = hex_lut[value & 0xFU];
        return buffer;
    }

private: // Device Descriptor
    static constexpr tusb_desc_device_t kDeviceDescriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = 0xA11C,
        .idProduct = LIBRMCS_USB_PID,
        .bcdDevice = 0x0300,
        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,
        .bNumConfigurations = 0x01,
    };

private: // Configuration Descriptor
    static constexpr uint8_t kItfNumDfu = 0U;
    static constexpr uint8_t kItfNumTotal = 1U;
    static constexpr size_t kConfigTotalLen = TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(1);

    static constexpr uint8_t kConfigurationDescriptorFs[] = {
        TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, kConfigTotalLen, 0, 100),
        TUD_DFU_DESCRIPTOR(kItfNumDfu, 1, 4, DFU_ATTR_CAN_DOWNLOAD, 1000, CFG_TUD_DFU_XFER_BUFSIZE),
    };
    static_assert(sizeof(kConfigurationDescriptorFs) == kConfigTotalLen);

private: // String Descriptor
    static constexpr std::array<uint8_t, 2> kLanguageId = {0x09, 0x04};
    static constexpr std::string_view kManufacturerString = "Alliance RoboMaster Team.";
    static constexpr std::string_view kProductString = "RMCS DFU Bootloader";
    static constexpr std::string_view kAlt0String = "Internal Flash";
    std::array<char, 43> serial_string_{"AF-0000-0000-0000-0000-0000-0000-0000-0000"};
    std::array<uint16_t, 128> descriptor_string_buffer_{};
};

UsbDescriptors& get_usb_descriptors();

} // namespace librmcs::firmware::usb
