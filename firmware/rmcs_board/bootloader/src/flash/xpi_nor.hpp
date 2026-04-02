#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <board.h>
#include <hpm_common.h>
#include <hpm_csr_regs.h>
#include <hpm_l1c_drv.h>
#include <hpm_romapi.h>
#include <hpm_romapi_xpi_def.h>
#include <hpm_romapi_xpi_nor_def.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>

#include "firmware/rmcs_board/bootloader/src/flash/layout.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::flash {

class XpiNor {
public:
    static XpiNor& instance() {
        static XpiNor flash;
        return flash;
    }

    uint32_t sector_size() const { return sector_size_; }

    void erase_sector(uintptr_t address) {
        utility::assert_debug((address % sector_size_) == 0U);

        const uint32_t irq_flags = disable_global_irq(CSR_MSTATUS_MIE_MASK);
        const hpm_stat_t status = rom_xpi_nor_erase_sector(
            BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_config_,
            to_flash_offset(address));
        restore_global_irq(irq_flags & CSR_MSTATUS_MIE_MASK);
        utility::assert_always(status == status_success);

        invalidate_range(address, sector_size_);
    }

    void program_word(uintptr_t address, uint32_t data) {
        program_words(address, std::span<const uint32_t>(&data, 1U));
    }

    void program_words(uintptr_t address, std::span<const uint32_t> data) {
        if (data.empty())
            return;

        utility::assert_debug((address & 0x3U) == 0U);
        utility::assert_debug(
            (reinterpret_cast<uintptr_t>(data.data()) & (alignof(uint32_t) - 1U)) == 0U);

        writeback_range(reinterpret_cast<uintptr_t>(data.data()), data.size_bytes());

        const uint32_t irq_flags = disable_global_irq(CSR_MSTATUS_MIE_MASK);
        const hpm_stat_t status = rom_xpi_nor_program(
            BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_config_, data.data(),
            to_flash_offset(address), static_cast<uint32_t>(data.size_bytes()));
        restore_global_irq(irq_flags & CSR_MSTATUS_MIE_MASK);
        utility::assert_always(status == status_success);

        invalidate_range(address, data.size_bytes());
    }

private:
    static uint32_t to_flash_offset(uintptr_t address) {
        utility::assert_debug(address >= kFlashBaseAddress);
        return static_cast<uint32_t>(address - BOARD_FLASH_BASE_ADDRESS);
    }

    static void invalidate_range(uintptr_t address, size_t size) {
        const uintptr_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(address);
        const uintptr_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(address + size);
        l1c_dc_invalidate(
            static_cast<uint32_t>(aligned_start),
            static_cast<uint32_t>(aligned_end - aligned_start));
    }

    static void writeback_range(uintptr_t address, size_t size) {
        const uintptr_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(address);
        const uintptr_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(address + size);
        l1c_dc_writeback(
            static_cast<uint32_t>(aligned_start),
            static_cast<uint32_t>(aligned_end - aligned_start));
    }

    XpiNor() {
        xpi_nor_config_option_t option{};
        option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
        option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
        option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

        utility::assert_always(
            rom_xpi_nor_auto_config(BOARD_APP_XPI_NOR_XPI_BASE, &nor_config_, &option)
            == status_success);
        utility::assert_always(
            rom_xpi_nor_get_property(
                BOARD_APP_XPI_NOR_XPI_BASE, &nor_config_, xpi_nor_property_sector_size,
                &sector_size_)
            == status_success);
        utility::assert_always(sector_size_ == kFlashSectorSize);
    }

    xpi_nor_config_t nor_config_{};
    uint32_t sector_size_ = 0U;
};

} // namespace librmcs::firmware::flash
