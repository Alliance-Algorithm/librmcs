#pragma once

#include <cstdint>

#include "firmware/rmcs_board/bootloader/src/flash/layout.hpp"
#include "firmware/rmcs_board/bootloader/src/flash/xpi_nor.hpp"
#include "firmware/rmcs_board/bootloader/src/utility/assert.hpp"

namespace librmcs::firmware::flash {

class Metadata {
public:
    static Metadata& get_instance() {
        static Metadata image_metadata;
        return image_metadata;
    }

    bool is_ready() const { return latest_valid_slot_state_ == DataSlotState::kReady; }
    bool is_flashing() const { return latest_valid_slot_state_ == DataSlotState::kFlashing; }

    uint32_t image_size() const { return latest_valid_slot_->image_size; }
    uint32_t image_crc32() const { return latest_valid_slot_->image_crc32; }

    void begin_flashing() {
        if (!latest_valid_slot_ || latest_valid_slot_state_ == DataSlotState::kFatal) {
            erase_and_rescan();
        } else if (latest_valid_slot_state_ == DataSlotState::kEmpty) {

        } else if (latest_valid_slot_state_ == DataSlotState::kFlashing) {
            return;
        } else if (latest_valid_slot_state_ == DataSlotState::kReady) {
            auto next_addr = reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot);
            if (next_addr >= kMetadataEndAddress)
                erase_and_rescan();
            else
                latest_valid_slot_ = reinterpret_cast<DataSlot*>(next_addr);
        }

        latest_valid_slot_->enter_flashing_state();
        latest_valid_slot_state_ = DataSlotState::kFlashing;
    }

    void finish_flashing(uint32_t size, uint32_t crc32) {
        utility::assert_debug(latest_valid_slot_state_ == DataSlotState::kFlashing);

        latest_valid_slot_->enter_ready_state(size, crc32);
        latest_valid_slot_state_ = DataSlotState::kReady;
    }

private:
    Metadata() { scan_latest_valid_slot(); }

    static constexpr uint32_t kFlashWordErased = 0xFFFFFFFFU;
    static constexpr uint32_t kImageMetadataMagic = 0x524D4353U; // "RMCS"
    static constexpr uint32_t kImageStateReady = 0x494D5244U;    // "IMRD"

    enum class DataSlotState : uint8_t {
        kFatal,
        kEmpty,
        kFlashing,
        kReady,
    };

    struct DataSlot {
        volatile uint32_t magic;
        volatile uint32_t image_state;
        volatile uint32_t image_size;
        volatile uint32_t image_crc32;

        DataSlotState read_state() const {
            switch (magic) {
            case kFlashWordErased:
                return (image_state == kFlashWordErased && image_size == kFlashWordErased
                        && image_crc32 == kFlashWordErased)
                         ? DataSlotState::kEmpty
                         : DataSlotState::kFatal;

            case kImageMetadataMagic:
                switch (image_state) {
                case kFlashWordErased:
                    return (image_size == kFlashWordErased && image_crc32 == kFlashWordErased)
                             ? DataSlotState::kFlashing
                             : DataSlotState::kFatal;
                case kImageStateReady:
                    return (image_size <= kAppMaxImageSize) ? DataSlotState::kReady
                                                            : DataSlotState::kFatal;
                default: return DataSlotState::kFatal;
                }

            default: return DataSlotState::kFatal;
            }
        }

        void enter_flashing_state() {
            utility::assert_debug(read_state() == DataSlotState::kEmpty);

            XpiNor::instance().program_word(
                reinterpret_cast<uintptr_t>(&magic), kImageMetadataMagic);

            utility::assert_always(read_state() == DataSlotState::kFlashing);
        }

        void enter_ready_state(uint32_t size, uint32_t crc32) {
            utility::assert_debug(read_state() == DataSlotState::kFlashing);

            auto& xpi_nor = XpiNor::instance();
            xpi_nor.program_word(reinterpret_cast<uintptr_t>(&image_size), size);
            xpi_nor.program_word(reinterpret_cast<uintptr_t>(&image_crc32), crc32);
            xpi_nor.program_word(reinterpret_cast<uintptr_t>(&image_state), kImageStateReady);

            utility::assert_always(read_state() == DataSlotState::kReady);
        }
    };

    void scan_latest_valid_slot() {
        static_assert(kMetadataStartAddress % sizeof(DataSlot) == 0U);
        static_assert(kMetadataEndAddress % sizeof(DataSlot) == 0U);

        latest_valid_slot_ = nullptr;
        latest_valid_slot_state_ = DataSlotState::kFatal;

        for (uintptr_t addr = kMetadataStartAddress; addr < kMetadataEndAddress;
             addr += sizeof(DataSlot)) {
            auto& slot = *reinterpret_cast<DataSlot*>(addr);
            switch (const auto state = slot.read_state()) {
            case DataSlotState::kFatal:
                latest_valid_slot_ = nullptr;
                latest_valid_slot_state_ = DataSlotState::kFatal;
                return;

            case DataSlotState::kEmpty:
                if (!latest_valid_slot_) {
                    latest_valid_slot_ = &slot;
                    latest_valid_slot_state_ = DataSlotState::kEmpty;
                }
                break;

            case DataSlotState::kFlashing:
            case DataSlotState::kReady:
                if (!latest_valid_slot_
                    || (reinterpret_cast<uintptr_t>(latest_valid_slot_) + sizeof(DataSlot)
                            == reinterpret_cast<uintptr_t>(&slot)
                        && latest_valid_slot_state_ == DataSlotState::kReady)) {
                    latest_valid_slot_ = &slot;
                    latest_valid_slot_state_ = state;
                } else {
                    latest_valid_slot_ = nullptr;
                    latest_valid_slot_state_ = DataSlotState::kFatal;
                    return;
                }
                break;
            }
        }
    }

    void erase_and_rescan() {
        XpiNor::instance().erase_sector(kMetadataStartAddress);
        scan_latest_valid_slot();
        utility::assert_always(
            latest_valid_slot_ != nullptr && latest_valid_slot_state_ == DataSlotState::kEmpty);
    }

    DataSlot* latest_valid_slot_ = nullptr;
    DataSlotState latest_valid_slot_state_ = DataSlotState::kFatal;
};

} // namespace librmcs::firmware::flash
