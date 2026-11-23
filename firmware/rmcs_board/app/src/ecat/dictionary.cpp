#include "firmware/rmcs_board/app/src/ecat/dictionary.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include <esc_coe.h>
}

namespace librmcs::firmware::ecat {

using core::protocol::ethercat::kProductCode;
using core::protocol::ethercat::kRevision;
using core::protocol::ethercat::kSoftwareVersionIndex;
using core::protocol::ethercat::kVendorId;

extern "C" {
alignas(uint32_t) uint8_t rxpdo[ProcessData::kSize]{};
alignas(uint32_t) uint8_t txpdo[ProcessData::kSize]{};
}

namespace {

class DictionaryEntries : public Dictionary {
public:
    static constexpr auto pdo(uint16_t object) {
        std::array<_objd, kEntries + 1> result{};
        result[0] = {0, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Mapped entries", kEntries, nullptr};
        for (uint8_t sub = 1; sub <= kEntries; ++sub) {
            const auto mapping = (static_cast<uint32_t>(object) << 16U)
                               | (static_cast<uint32_t>(sub) << 8U) | entry_bits(sub - 1);
            result[sub] = {sub, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Mapping", mapping, nullptr};
        }
        return result;
    }

    static constexpr auto data(uint8_t* buffer, uint16_t access) {
        std::array<_objd, kEntries + 1> result{};
        result[0] = {0, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Entries", kEntries, nullptr};
        for (uint8_t sub = 1; sub <= kEntries; ++sub) {
            result[sub] = {
                sub,
                DTYPE_OCTET_STRING,
                entry_bits(sub - 1),
                access,
                "Process data",
                0,
                buffer + (sub - 1) * kEntrySize};
        }
        return result;
    }
};

constexpr auto kRxMapping = DictionaryEntries::pdo(Dictionary::kOutput);
constexpr auto kTxMapping = DictionaryEntries::pdo(Dictionary::kInput);
constexpr auto kOutputData = DictionaryEntries::data(rxpdo, ATYPE_RO | ATYPE_RXPDO);
constexpr auto kInputData = DictionaryEntries::data(txpdo, ATYPE_RO | ATYPE_TXPDO);

constexpr _objd kDeviceType[] = {
    {0, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Device type", 0, nullptr},
};
constinit char version[] = LIBRMCS_PROJECT_VERSION_STRING;
constexpr _objd kVersion[] = {
    {0, DTYPE_VISIBLE_STRING, 8 * (sizeof(version) - 1), ATYPE_RO, "Software version", 0, version},
};
constexpr _objd kIdentity[] = {
    {0,  DTYPE_UNSIGNED8,  8, ATYPE_RO,   "Entries",            4, nullptr},
    {1, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Vendor ID",    kVendorId, nullptr},
    {2, DTYPE_UNSIGNED32, 32, ATYPE_RO,   "Product", kProductCode, nullptr},
    {3, DTYPE_UNSIGNED32, 32, ATYPE_RO,  "Revision",    kRevision, nullptr},
    {4, DTYPE_UNSIGNED32, 32, ATYPE_RO,    "Serial",            0, nullptr},
};
constexpr _objd kSmTypes[] = {
    {0, DTYPE_UNSIGNED8, 8, ATYPE_RO,     "Entries", 4, nullptr},
    {1, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Mailbox out", 1, nullptr},
    {2, DTYPE_UNSIGNED8, 8, ATYPE_RO,  "Mailbox in", 2, nullptr},
    {3, DTYPE_UNSIGNED8, 8, ATYPE_RO,     "Outputs", 3, nullptr},
    {4, DTYPE_UNSIGNED8, 8, ATYPE_RO,      "Inputs", 4, nullptr},
};
constexpr _objd kRxAssignment[] = {
    {0,  DTYPE_UNSIGNED8,  8, ATYPE_RO, "Entries",                  1, nullptr},
    {1, DTYPE_UNSIGNED16, 16, ATYPE_RO,   "RxPDO", Dictionary::kRxPdo, nullptr},
};
constexpr _objd kTxAssignment[] = {
    {0,  DTYPE_UNSIGNED8,  8, ATYPE_RO, "Entries",                  1, nullptr},
    {1, DTYPE_UNSIGNED16, 16, ATYPE_RO,   "TxPDO", Dictionary::kTxPdo, nullptr},
};

} // namespace

extern "C" const _objectlist SDOobjects[] = {
    {               0x1000,    OTYPE_VAR,                    0, 0,       "Device type",        kDeviceType},
    {kSoftwareVersionIndex,    OTYPE_VAR,                    0, 0,  "Software version",           kVersion},
    {               0x1018, OTYPE_RECORD,                    4, 0,          "Identity",          kIdentity},
    {   Dictionary::kRxPdo, OTYPE_RECORD, Dictionary::kEntries, 0,             "RxPDO",  kRxMapping.data()},
    {   Dictionary::kTxPdo, OTYPE_RECORD, Dictionary::kEntries, 0,             "TxPDO",  kTxMapping.data()},
    {               0x1c00,  OTYPE_ARRAY,                    4, 0, "SyncManager types",           kSmTypes},
    {        RX_PDO_OBJIDX,  OTYPE_ARRAY,                    1, 0,  "RxPDO assignment",      kRxAssignment},
    {        TX_PDO_OBJIDX,  OTYPE_ARRAY,                    1, 0,  "TxPDO assignment",      kTxAssignment},
    {   Dictionary::kInput, OTYPE_RECORD, Dictionary::kEntries, 0,            "Inputs",  kInputData.data()},
    {  Dictionary::kOutput, OTYPE_RECORD, Dictionary::kEntries, 0,           "Outputs", kOutputData.data()},
    {               0xffff,         0xff,                 0xff, 0,             nullptr,            nullptr},
};

} // namespace librmcs::firmware::ecat
