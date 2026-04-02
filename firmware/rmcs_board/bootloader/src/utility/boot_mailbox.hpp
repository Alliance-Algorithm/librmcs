#pragma once

#include <array>
#include <cstdint>

#include <hpm_common.h>
#include <hpm_ppor_drv.h>
#include <hpm_soc.h>
#include <hpm_sysctl_drv.h>

#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::boot {

class BootMailbox {
public:
    static void clear() { write_pair(0U, 0U); }

    static bool consume_enter_dfu_request() {
        const auto values = read_pair();
        clear();
        return values[0] == kMailboxMagic && values[1] == kMailboxRequestEnterDfu;
    }

    [[noreturn]] static void reboot() {
        ppor_reset_mask_set_source_enable(HPM_PPOR, ppor_reset_software);
        ppor_reset_set_hot_reset_enable(HPM_PPOR, ppor_reset_software);
        ppor_sw_reset(HPM_PPOR, 10U);
        while (true)
            ;
    }

private:
    static constexpr uint32_t kMailboxMagic = 0x524D4353U;           // "RMCS"
    static constexpr uint32_t kMailboxRequestEnterDfu = 0x44465530U; // "DFU0"
    static constexpr uint8_t kMagicGprIndex = 12U;

    static std::array<uint32_t, 2> read_pair() {
        std::array<uint32_t, 2> values{};
        utility::assert_always(
            sysctl_cpu0_get_gpr(HPM_SYSCTL, kMagicGprIndex, values.size(), values.data())
            == status_success);
        return values;
    }

    static void write_pair(uint32_t magic, uint32_t request) {
        uint32_t values[2]{magic, request};
        utility::assert_always(
            sysctl_cpu0_set_gpr(HPM_SYSCTL, kMagicGprIndex, 2U, values, false) == status_success);
    }
};

} // namespace librmcs::firmware::boot
