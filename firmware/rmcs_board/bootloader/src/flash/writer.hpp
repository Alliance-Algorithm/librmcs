#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "firmware/rmcs_board/bootloader/include/tusb_config.h"
#include "firmware/rmcs_board/bootloader/src/flash/layout.hpp"
#include "firmware/rmcs_board/bootloader/src/flash/xpi_nor.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::flash {

class Writer {
public:
    static constexpr uint32_t kTransferBlockSize = CFG_TUD_DFU_XFER_BUFSIZE;

    void begin_session() { clear_active_sector(); }

    void finish_session() {
        if (!has_active_sector_)
            return;

        commit_active_sector_if_needed();
        clear_active_sector();
    }

    void abort_session() { clear_active_sector(); }

    void write(uintptr_t address, std::span<const std::byte> data) {
        utility::assert_debug(!data.empty());
        utility::assert_debug(address >= kAppStartAddress && address < kAppEndAddress);
        utility::assert_debug(
            static_cast<uint64_t>(address) + data.size() <= static_cast<uint64_t>(kAppEndAddress));

        size_t input_offset = 0U;
        while (input_offset < data.size()) {
            const uintptr_t write_address = address + input_offset;
            const uintptr_t sector_address =
                write_address - (write_address % XpiNor::instance().sector_size());
            activate_sector(sector_address);

            const uintptr_t sector_offset = write_address - active_sector_address_;
            const size_t writable =
                static_cast<size_t>(XpiNor::instance().sector_size() - sector_offset);
            const size_t chunk_size = std::min(writable, data.size() - input_offset);

            utility::assert_debug(sector_offset == buffered_size_);
            std::memcpy(
                writable_sector_buffer_bytes().data() + sector_offset, data.data() + input_offset,
                chunk_size);
            advance_buffer(sector_offset, chunk_size);

            input_offset += chunk_size;
            if ((sector_offset + chunk_size) == XpiNor::instance().sector_size()) {
                commit_active_sector_if_needed();
                clear_active_sector();
            }
        }
    }

private:
    static constexpr size_t kSectorBufferCapacity = kFlashSectorSize;
    static constexpr size_t kSectorBufferWordCount = kSectorBufferCapacity / sizeof(uint32_t);
    static constexpr uintptr_t kInvalidAddress = ~static_cast<uintptr_t>(0U);

    static std::span<std::byte> writable_sector_buffer_bytes() {
        return std::as_writable_bytes(std::span<uint32_t>(sector_buffer_));
    }

    static std::span<const std::byte> sector_buffer_bytes() {
        return std::as_bytes(std::span<const uint32_t>(sector_buffer_));
    }

    void activate_sector(uintptr_t sector_address) {
        if (has_active_sector_ && active_sector_address_ == sector_address)
            return;

        if (has_active_sector_) {
            commit_active_sector_if_needed();
            clear_active_sector();
        }

        utility::assert_debug((sector_address % kSectorBufferCapacity) == 0U);
        has_active_sector_ = true;
        active_sector_address_ = sector_address;
        buffered_size_ = 0U;
    }

    void advance_buffer(uintptr_t offset, size_t size) {
        utility::assert_debug(has_active_sector_);
        utility::assert_debug(size > 0U);
        utility::assert_debug(offset <= static_cast<uintptr_t>(SIZE_MAX) - size);

        buffered_size_ = std::max(buffered_size_, static_cast<size_t>(offset) + size);
    }

    bool has_buffered_data() const { return has_active_sector_ && buffered_size_ > 0U; }

    void clear_active_sector() {
        has_active_sector_ = false;
        active_sector_address_ = kInvalidAddress;
        buffered_size_ = 0U;
    }

    void commit_active_sector_if_needed() {
        utility::assert_debug(has_active_sector_);
        if (!has_buffered_data())
            return;

        const auto* flash_ptr = reinterpret_cast<const void*>(active_sector_address_);
        const bool is_same =
            std::memcmp(flash_ptr, sector_buffer_bytes().data(), buffered_size_) == 0;
        if (!is_same) {
            auto& xpi_nor = XpiNor::instance();
            xpi_nor.erase_sector(active_sector_address_);
            program_buffer(active_sector_address_, buffered_size_);
        }

        buffered_size_ = 0U;
    }

    static void program_buffer(uintptr_t address, size_t size) {
        utility::assert_debug((address & 0x3U) == 0U);

        const size_t full_word_count = size / sizeof(uint32_t);
        if (full_word_count > 0U) {
            XpiNor::instance().program_words(
                address, std::span<const uint32_t>(sector_buffer_.data(), full_word_count));
        }

        const size_t full_word_bytes = full_word_count * sizeof(uint32_t);
        const size_t tail_size = size - full_word_bytes;
        if (tail_size == 0U)
            return;

        uint32_t tail_word = 0xFFFFFFFFU;
        std::memcpy(&tail_word, sector_buffer_bytes().data() + full_word_bytes, tail_size);
        XpiNor::instance().program_word(address + full_word_bytes, tail_word);
    }

    static_assert((kSectorBufferCapacity % sizeof(uint32_t)) == 0U);
    alignas(4) static inline std::array<uint32_t, kSectorBufferWordCount> sector_buffer_{};

    bool has_active_sector_ = false;
    uintptr_t active_sector_address_ = kInvalidAddress;
    size_t buffered_size_ = 0U;
};

} // namespace librmcs::firmware::flash
