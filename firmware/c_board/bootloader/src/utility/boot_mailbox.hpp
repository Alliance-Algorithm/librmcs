#pragma once

#include <cstdint>
#include <type_traits>

namespace librmcs::firmware::utility {

struct BootMailbox {
    static constexpr uint32_t kMailboxMagic = 0x524D4353;           // "RMCS"
    static constexpr uint32_t kMailboxRequestEnterDfu = 0x44465530; // "DFU0"

    volatile uint32_t magic;
    volatile uint32_t request;

    void clear() {
        magic = 0;
        request = 0;
    }

    bool consume_enter_dfu_request() {
        const bool requested = magic == kMailboxMagic && request == kMailboxRequestEnterDfu;
        clear();
        return requested;
    }
};
inline BootMailbox boot_mailbox __attribute__((section(".boot_mailbox"), aligned(4), used));

static_assert(std::is_standard_layout_v<BootMailbox> && sizeof(BootMailbox) <= 64);

} // namespace librmcs::firmware::utility
