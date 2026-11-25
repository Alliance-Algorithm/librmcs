#include "firmware/rmcs_board/app/src/ecat/esc.hpp"

#include <cstddef>
#include <cstdint>

#include <hpm_esc_drv.h>
#include <hpm_esc_regs.h>
#include <hpm_soc.h>

extern "C" {
#include <esc.h>
}

#include "ethercat_phy.hpp"
#include "firmware/rmcs_board/app/src/ecat/sii.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"

namespace librmcs::firmware::ecat {

bool Esc::start() {
    if (!board::EthercatPhy::start_clock())
        return false;

    constexpr uint32_t eeprom_timeout_quarter_us = 20'000 * 4;
    const auto start = timer::Timer::timestamp_quarter_us();
    do {
        service_eeprom();
        if ((HPM_ESC->ESC_DL_STAT & ESC_ESC_DL_STAT_EPLC_MASK) != 0)
            return board::EthercatPhy::start_links();
    } while (timer::Timer::timestamp_quarter_us() - start < eeprom_timeout_quarter_us);
    return false;
}

void Esc::service_eeprom() {
    constexpr auto image = Sii::make_image();
    if ((HPM_ESC->EEPROM_CTRL_STAT & ESC_EEPROM_CTRL_STAT_BUSY_MASK) == 0)
        return;

    const auto command = static_cast<esc_eeprom_cmd_t>(esc_get_eeprom_cmd(HPM_ESC));
    bool error = false;
    switch (command) {
    case esc_eeprom_reload_cmd:
        // Station alias and enhanced-link flags are all zero in our SII.
        esc_write_eeprom_data(HPM_ESC, 0);
        break;
    case esc_eeprom_read_cmd: {
        constexpr size_t words_per_read = sizeof(uint64_t) / sizeof(uint16_t);
        const auto offset = esc_get_eeprom_word_address(HPM_ESC);
        if (offset > image.size() - words_per_read) {
            error = true;
            break;
        }

        uint64_t data = 0;
        for (size_t i = 0; i < words_per_read; ++i)
            data |= static_cast<uint64_t>(image[offset + i]) << (16U * i);
        HPM_ESC->EEPROM_DATA = data;
        break;
    }
    default: error = true; break; // Read-only SII.
    }
    esc_eeprom_emulation_ack(HPM_ESC, command, error, false);
}

void Esc::read(uint16_t address, void* data, uint16_t size) {
    // The PDI bus requires aligned 32-bit transactions even for byte registers.
    auto* destination = static_cast<uint8_t*>(data);
    for (uint32_t current = address; current < static_cast<uint32_t>(address) + size;) {
        const uint32_t aligned = current & ~3U;
        const auto word = *reinterpret_cast<const volatile uint32_t*>(HPM_ESC_BASE + aligned);
        do {
            *destination++ = static_cast<uint8_t>(word >> (8U * (current++ - aligned)));
        } while (current - aligned < 4 && current < static_cast<uint32_t>(address) + size);
    }
    ESCvar.ALevent = HPM_ESC->AL_EVT_REQ;
}

void Esc::write(uint16_t address, const void* data, uint16_t size) {
    const auto* source = static_cast<const uint8_t*>(data);
    const auto end = static_cast<uint32_t>(address) + size;
    for (uint32_t current = address; current < end;) {
        const uint32_t aligned = current & ~3U;
        auto* target = reinterpret_cast<volatile uint32_t*>(HPM_ESC_BASE + aligned);
        uint32_t word = current == aligned && end - current >= 4 ? 0 : *target;
        do {
            const auto shift = 8U * (current++ - aligned);
            word = (word & ~(0xffU << shift)) | (static_cast<uint32_t>(*source++) << shift);
        } while (current - aligned < 4 && current < end);
        *target = word;
    }
    ESCvar.ALevent = HPM_ESC->AL_EVT_REQ;
}

extern "C" {
void ESC_read(uint16_t address, void* buf, uint16_t len) { Esc::read(address, buf, len); }
void ESC_write(uint16_t address, void* buf, uint16_t len) { Esc::write(address, buf, len); }
// Esc::start initializes the hardware before SOES starts.
void ESC_init(const esc_cfg_t*) {}
}

} // namespace librmcs::firmware::ecat
