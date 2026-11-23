#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include <esc.h>
#include <esc_coe.h>
}

#include "firmware/rmcs_board/app/src/ecat/dictionary.hpp"

namespace librmcs::firmware::ecat {

class Sii {
public:
    static constexpr size_t kWordCount = 1024;
    using Image = std::array<uint16_t, kWordCount>;

    static consteval Image make_image() {
        Sii sii;
        auto& config = sii.image_;
        config[0] = 0x0c80; // On-chip PDI; enhanced link detection is disabled.
        config[1] = 0x6681;
        config[7] = sii.checksum();
        sii.write_id(8, core::protocol::ethercat::kVendorId);
        sii.write_id(10, core::protocol::ethercat::kProductCode);
        sii.write_id(12, core::protocol::ethercat::kRevision);
        config[0x18] = MBX0_sma_b;
        config[0x19] = MBX0_sml_b;
        config[0x1a] = MBX1_sma_b;
        config[0x1b] = MBX1_sml_b;
        config[0x1c] = MBX0_sma;
        config[0x1d] = MBX0_sml;
        config[0x1e] = MBX1_sma;
        config[0x1f] = MBX1_sml;
        config[0x20] = 4; // CoE
        config[0x3e] = sizeof(Image) / 128 - 1;
        config[0x3f] = 1;

        constexpr size_t name_length = sizeof(Dictionary::kName) - 1;
        sii.append(10); // Strings
        sii.append(1 + (name_length + 1) / 2);
        sii.append(1 | (name_length << 8));
        for (size_t i = 0; i < name_length; i += 2) {
            sii.append(
                static_cast<uint16_t>(Dictionary::kName[i])
                | (static_cast<uint16_t>(Dictionary::kName[i + 1]) << 8));
        }

        sii.append(30);                   // General
        sii.append(16);
        config[sii.cursor_ + 1] = 0x0100; // Name: string 1
        config[sii.cursor_ + 2] = 0x0300; // CoE: SDO and SDO Info, no Complete Access
        sii.cursor_ += 16;

        sii.append(40);     // FMMU
        sii.append(2);
        sii.append(0x0201); // Outputs, inputs
        sii.append(0xffff); // Unused

        sii.append(41);     // SyncManager
        sii.append(16);
        sii.sm(MBX0_sma, MBX0_sml, MBX0_smc, 1);
        sii.sm(MBX1_sma, MBX1_sml, MBX1_smc, 2);
        sii.sm(SM2_sma, ProcessData::kSize, SM2_smc, 3);
        sii.sm(SM3_sma, ProcessData::kSize, SM3_smc, 4);

        sii.pdo(50, Dictionary::kTxPdo, 3, Dictionary::kInput);
        sii.pdo(51, Dictionary::kRxPdo, 2, Dictionary::kOutput);
        sii.append(0xffff);
        return config;
    }

private:
    void constexpr write_id(size_t offset, uint32_t id) {
        image_[offset] = static_cast<uint16_t>(id);
        image_[offset + 1] = static_cast<uint16_t>(id >> 16U);
    }

    void constexpr append(uint16_t word) { image_[cursor_++] = word; }

    void constexpr sm(uint16_t address, uint16_t size, uint8_t control, uint8_t type) {
        append(address);
        append(size);
        append(control);
        append(static_cast<uint16_t>(1 | (type << 8)));
    }

    void constexpr pdo(uint16_t category, uint16_t index, uint8_t sm_index, uint16_t object) {
        append(category);
        append(4 * (1 + Dictionary::kEntries));
        append(index);
        append(static_cast<uint16_t>(Dictionary::kEntries | (sm_index << 8)));
        append(0);
        append(0x0011); // Mandatory, fixed mapping
        for (uint8_t sub = 1; sub <= Dictionary::kEntries; ++sub) {
            append(object);
            append(sub);
            append(
                static_cast<uint16_t>(DTYPE_OCTET_STRING | (Dictionary::entry_bits(sub - 1) << 8)));
            append(0);
        }
    }

    constexpr uint8_t checksum() const {
        uint8_t crc = 0xff;
        for (size_t i = 0; i < 14; ++i) {
            crc ^= static_cast<uint8_t>(image_[i / 2] >> (8U * (i % 2)));
            for (unsigned bit = 0; bit < 8; ++bit)
                crc = static_cast<uint8_t>((crc << 1U) ^ ((crc & 0x80U) != 0 ? 7U : 0U));
        }
        return crc;
    }

    Image image_{};
    size_t cursor_ = 0x40;
};

} // namespace librmcs::firmware::ecat
