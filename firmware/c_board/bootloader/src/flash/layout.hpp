#pragma once

#include <cstddef>
#include <cstdint>

#include <main.h>

namespace librmcs::firmware::flash {

inline constexpr uintptr_t kAppStartAddress = 0x08010000U;
inline constexpr uintptr_t kAppEndAddress = 0x08100000U; // exclusive
inline constexpr size_t kAppMaxImageSize = kAppEndAddress - kAppStartAddress;

struct SectorRange {
    uint32_t start;
    uint32_t end; // exclusive
    uint32_t sector;
};

inline constexpr size_t kAppSectorCount = 8U;
inline constexpr SectorRange kAppSectors[kAppSectorCount] = {
    {.start = 0x08010000U, .end = 0x08020000U,  .sector = FLASH_SECTOR_4},
    {.start = 0x08020000U, .end = 0x08040000U,  .sector = FLASH_SECTOR_5},
    {.start = 0x08040000U, .end = 0x08060000U,  .sector = FLASH_SECTOR_6},
    {.start = 0x08060000U, .end = 0x08080000U,  .sector = FLASH_SECTOR_7},
    {.start = 0x08080000U, .end = 0x080A0000U,  .sector = FLASH_SECTOR_8},
    {.start = 0x080A0000U, .end = 0x080C0000U,  .sector = FLASH_SECTOR_9},
    {.start = 0x080C0000U, .end = 0x080E0000U, .sector = FLASH_SECTOR_10},
    {.start = 0x080E0000U, .end = 0x08100000U, .sector = FLASH_SECTOR_11},
};

} // namespace librmcs::firmware::flash
