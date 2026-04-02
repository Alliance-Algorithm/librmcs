#pragma once

#include <cstdint>

#include <main.h>

#include "firmware/c_board/bootloader/src/flash/layout.hpp"
#include "firmware/c_board/bootloader/src/flash/unlock_guard.hpp"
#include "firmware/c_board/bootloader/src/utility/assert.hpp"

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

    static constexpr uintptr_t kMetadataStartAddress = 0x0800C000U;
    static constexpr uintptr_t kMetadataEndAddress = 0x08010000U;

    static constexpr uint32_t kFlashWordErased = 0xFFFFFFFF;

    static constexpr uint32_t kImageMetadataMagic = 0x524D4353; // "RMCS"
    static constexpr uint32_t kImageStateReady = 0x494D5244;    // "IMRD"

    enum class DataSlotState : uint8_t {
        kFatal,    // Unexpected, the entire flash sector needed to be erased
        kEmpty,    // A clean slot that can be written to
        kFlashing, // The program is being flashed
        kReady,    // The program is ready (only the data in the last slot is valid)
    };

    struct DataSlot {
        volatile uint32_t magic;
        volatile uint32_t image_state;
        volatile uint32_t image_size;
        volatile uint32_t image_crc32;

        DataSlotState read_state() const {
            switch (magic) {
            case kFlashWordErased: {
                return (image_state == kFlashWordErased && image_size == kFlashWordErased
                        && image_crc32 == kFlashWordErased)
                         ? DataSlotState::kEmpty
                         : DataSlotState::kFatal;
            }
            case kImageMetadataMagic: {
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
            }
            default: return DataSlotState::kFatal;
            }
        }

        void enter_flashing_state() {
            utility::assert_debug(read_state() == DataSlotState::kEmpty);

            {
                const auto guard = UnlockGuard();
                utility::assert_always(
                    HAL_FLASH_Program(
                        FLASH_TYPEPROGRAM_WORD, reinterpret_cast<uintptr_t>(&magic),
                        kImageMetadataMagic)
                    == HAL_OK);
            }

            utility::assert_always(read_state() == DataSlotState::kFlashing);
        }

        void enter_ready_state(uint32_t size, uint32_t crc32) {
            utility::assert_debug(read_state() == DataSlotState::kFlashing);

            {
                const auto guard = UnlockGuard();
                utility::assert_always(
                    HAL_FLASH_Program(
                        FLASH_TYPEPROGRAM_WORD, reinterpret_cast<uintptr_t>(&image_size), size)
                        == HAL_OK
                    && HAL_FLASH_Program(
                           FLASH_TYPEPROGRAM_WORD, reinterpret_cast<uintptr_t>(&image_crc32), crc32)
                           == HAL_OK
                    && HAL_FLASH_Program(
                           FLASH_TYPEPROGRAM_WORD, reinterpret_cast<uintptr_t>(&image_state),
                           kImageStateReady)
                           == HAL_OK);
            }

            utility::assert_always(read_state() == DataSlotState::kReady);
        }
    };

    void scan_latest_valid_slot() {
        static_assert(kMetadataStartAddress % sizeof(DataSlot) == 0);
        static_assert(kMetadataEndAddress % sizeof(DataSlot) == 0);

        latest_valid_slot_ = nullptr;
        latest_valid_slot_state_ = DataSlotState::kFatal;

        for (uintptr_t addr = kMetadataStartAddress; addr < kMetadataEndAddress;
             addr += sizeof(DataSlot)) {
            auto& slot = *reinterpret_cast<DataSlot*>(addr);
            switch (const auto state = slot.read_state()) {

            case DataSlotState::kFatal: {
                latest_valid_slot_ = nullptr;
                latest_valid_slot_state_ = DataSlotState::kFatal;
                return;
            }

            case DataSlotState::kEmpty: {
                if (!latest_valid_slot_) {
                    latest_valid_slot_ = &slot;
                    latest_valid_slot_state_ = DataSlotState::kEmpty;
                }
                break;
            }

            case DataSlotState::kFlashing:
            case DataSlotState::kReady: {
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
    }

    void erase_and_rescan() {
        FLASH_EraseInitTypeDef erase{};
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        erase.Sector = FLASH_SECTOR_3;
        erase.NbSectors = 1;

        uint32_t sector_error;
        {
            const auto guard = UnlockGuard();
            utility::assert_always(HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK);
        }

        scan_latest_valid_slot();
        utility::assert_always(
            latest_valid_slot_ != nullptr && latest_valid_slot_state_ == DataSlotState::kEmpty);
    }

    DataSlot* latest_valid_slot_ = nullptr;
    DataSlotState latest_valid_slot_state_ = DataSlotState::kFatal;
};

} // namespace librmcs::firmware::flash
