#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "core/src/protocol/ethercat/process_data.hpp"

namespace librmcs::firmware::ecat {

class Dictionary {
    using ProcessData = core::protocol::ethercat::ProcessData;

public:
    static constexpr char kName[] = "RMCS Board Ultra";
    static constexpr uint16_t kOutput = 0x7000;
    static constexpr uint16_t kInput = 0x6000;
    static constexpr uint16_t kRxPdo = 0x1600;
    static constexpr uint16_t kTxPdo = 0x1a00;
    // A mapping entry encodes the bit count in one byte.
    static constexpr size_t kEntrySize = UINT8_MAX / 8;
    static constexpr uint8_t kEntries = (ProcessData::kSize + kEntrySize - 1) / kEntrySize;

    static constexpr uint8_t entry_bits(size_t entry) {
        const auto offset = entry * kEntrySize;
        return static_cast<uint8_t>(8 * std::min(kEntrySize, ProcessData::kSize - offset));
    }
};

} // namespace librmcs::firmware::ecat
