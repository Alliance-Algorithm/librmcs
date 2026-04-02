#pragma once

#include <cstddef>
#include <cstdint>

#include <board.h>

namespace librmcs::firmware::flash {

inline constexpr uintptr_t kFlashBaseAddress = BOARD_FLASH_BASE_ADDRESS;
inline constexpr uintptr_t kMetadataStartAddress = 0x8001F000U;
inline constexpr uintptr_t kMetadataEndAddress = 0x80020000U;
inline constexpr uintptr_t kAppStartAddress = 0x80020000U;
inline constexpr uintptr_t kAppEntryAddress = kAppStartAddress + sizeof(uint32_t);
inline constexpr uintptr_t kAppEndAddress = BOARD_FLASH_BASE_ADDRESS + BOARD_FLASH_SIZE;
inline constexpr size_t kAppMaxImageSize = kAppEndAddress - kAppStartAddress;
inline constexpr uint32_t kFlashSectorSize = 4096U;

static_assert(kMetadataEndAddress == kAppStartAddress);
static_assert((kMetadataStartAddress % kFlashSectorSize) == 0U);
static_assert((kAppStartAddress % kFlashSectorSize) == 0U);
static_assert((kAppEndAddress % kFlashSectorSize) == 0U);

} // namespace librmcs::firmware::flash
