#include "firmware/rmcs_board/app/src/ecat/dictionary.hpp"

#include <array>
#include <cstdint>

extern "C" {
#include <esc_coe.h>
}

#include "core/src/protocol/ethercat/process_data.hpp"

namespace librmcs::firmware::ecat {

using core::protocol::ethercat::kProductCode;
using core::protocol::ethercat::kRevision;
using core::protocol::ethercat::kSoftwareVersionIndex;
using core::protocol::ethercat::kVendorId;
using core::protocol::ethercat::ProcessData;

extern "C" {
alignas(uint32_t) uint8_t rxpdo[ProcessData::kSize]{};
alignas(uint32_t) uint8_t txpdo[ProcessData::kSize]{};
}

namespace {

class DictionaryEntries : public Dictionary {
public:
    static constexpr auto pdo(uint16_t object) {
        std::array<_objd, kEntries + 1> result{};
        result[0] = {
            .subindex = 0,
            .datatype = DTYPE_UNSIGNED8,
            .bitlength = 8,
            .flags = ATYPE_RO,
            .name = "Mapped entries",
            .value = kEntries,
            .data = nullptr};
        for (uint8_t sub = 1; sub <= kEntries; ++sub) {
            const auto mapping = (static_cast<uint32_t>(object) << 16U)
                               | (static_cast<uint32_t>(sub) << 8U) | entry_bits(sub - 1);
            result[sub] = {
                .subindex = sub,
                .datatype = DTYPE_UNSIGNED32,
                .bitlength = 32,
                .flags = ATYPE_RO,
                .name = "Mapping",
                .value = mapping,
                .data = nullptr};
        }
        return result;
    }

    static constexpr auto data(uint8_t* buffer, uint16_t access) {
        std::array<_objd, kEntries + 1> result{};
        result[0] = {
            .subindex = 0,
            .datatype = DTYPE_UNSIGNED8,
            .bitlength = 8,
            .flags = ATYPE_RO,
            .name = "Entries",
            .value = kEntries,
            .data = nullptr};
        for (uint8_t sub = 1; sub <= kEntries; ++sub) {
            result[sub] = {
                .subindex = sub,
                .datatype = DTYPE_OCTET_STRING,
                .bitlength = entry_bits(sub - 1),
                .flags = access,
                .name = "Process data",
                .value = 0,
                .data = buffer + ((sub - 1) * kEntrySize)};
        }
        return result;
    }
};

constexpr auto kRxMapping = DictionaryEntries::pdo(Dictionary::kOutput);
constexpr auto kTxMapping = DictionaryEntries::pdo(Dictionary::kInput);
constexpr auto kOutputData = DictionaryEntries::data(rxpdo, ATYPE_RO | ATYPE_RXPDO);
constexpr auto kInputData = DictionaryEntries::data(txpdo, ATYPE_RO | ATYPE_TXPDO);

constexpr _objd kDeviceType[] = {
    {.subindex = 0,
     .datatype = DTYPE_UNSIGNED32,
     .bitlength = 32,
     .flags = ATYPE_RO,
     .name = "Device type",
     .value = 0,
     .data = nullptr},
};
constinit char version[] = LIBRMCS_PROJECT_VERSION_STRING;
constexpr _objd kVersion[] = {
    {.subindex = 0,
     .datatype = DTYPE_VISIBLE_STRING,
     .bitlength = 8 * (sizeof(version) - 1),
     .flags = ATYPE_RO,
     .name = "Software version",
     .value = 0,
     .data = version},
};
constexpr _objd kIdentity[] = {
    {.subindex = 0,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Entries",
     .value = 4,
     .data = nullptr},
    {.subindex = 1,
     .datatype = DTYPE_UNSIGNED32,
     .bitlength = 32,
     .flags = ATYPE_RO,
     .name = "Vendor ID",
     .value = kVendorId,
     .data = nullptr},
    {.subindex = 2,
     .datatype = DTYPE_UNSIGNED32,
     .bitlength = 32,
     .flags = ATYPE_RO,
     .name = "Product",
     .value = kProductCode,
     .data = nullptr},
    {.subindex = 3,
     .datatype = DTYPE_UNSIGNED32,
     .bitlength = 32,
     .flags = ATYPE_RO,
     .name = "Revision",
     .value = kRevision,
     .data = nullptr},
    {.subindex = 4,
     .datatype = DTYPE_UNSIGNED32,
     .bitlength = 32,
     .flags = ATYPE_RO,
     .name = "Serial",
     .value = 0,
     .data = nullptr},
};
constexpr _objd kSmTypes[] = {
    {.subindex = 0,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Entries",
     .value = 4,
     .data = nullptr},
    {.subindex = 1,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Mailbox out",
     .value = 1,
     .data = nullptr},
    {.subindex = 2,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Mailbox in",
     .value = 2,
     .data = nullptr},
    {.subindex = 3,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Outputs",
     .value = 3,
     .data = nullptr},
    {.subindex = 4,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Inputs",
     .value = 4,
     .data = nullptr},
};
constexpr _objd kRxAssignment[] = {
    {.subindex = 0,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Entries",
     .value = 1,
     .data = nullptr},
    {.subindex = 1,
     .datatype = DTYPE_UNSIGNED16,
     .bitlength = 16,
     .flags = ATYPE_RO,
     .name = "RxPDO",
     .value = Dictionary::kRxPdo,
     .data = nullptr},
};
constexpr _objd kTxAssignment[] = {
    {.subindex = 0,
     .datatype = DTYPE_UNSIGNED8,
     .bitlength = 8,
     .flags = ATYPE_RO,
     .name = "Entries",
     .value = 1,
     .data = nullptr},
    {.subindex = 1,
     .datatype = DTYPE_UNSIGNED16,
     .bitlength = 16,
     .flags = ATYPE_RO,
     .name = "TxPDO",
     .value = Dictionary::kTxPdo,
     .data = nullptr},
};

} // namespace

extern "C" const _objectlist SDOobjects[] = {
    {               .index = 0x1000,
     .objtype = OTYPE_VAR,
     .maxsub = 0,
     .pad1 = 0,
     .name = "Device type",
     .objdesc = kDeviceType       },
    {.index = kSoftwareVersionIndex,
     .objtype = OTYPE_VAR,
     .maxsub = 0,
     .pad1 = 0,
     .name = "Software version",
     .objdesc = kVersion          },
    {               .index = 0x1018,
     .objtype = OTYPE_RECORD,
     .maxsub = 4,
     .pad1 = 0,
     .name = "Identity",
     .objdesc = kIdentity         },
    {   .index = Dictionary::kRxPdo,
     .objtype = OTYPE_RECORD,
     .maxsub = Dictionary::kEntries,
     .pad1 = 0,
     .name = "RxPDO",
     .objdesc = kRxMapping.data() },
    {   .index = Dictionary::kTxPdo,
     .objtype = OTYPE_RECORD,
     .maxsub = Dictionary::kEntries,
     .pad1 = 0,
     .name = "TxPDO",
     .objdesc = kTxMapping.data() },
    {               .index = 0x1c00,
     .objtype = OTYPE_ARRAY,
     .maxsub = 4,
     .pad1 = 0,
     .name = "SyncManager types",
     .objdesc = kSmTypes          },
    {        .index = RX_PDO_OBJIDX,
     .objtype = OTYPE_ARRAY,
     .maxsub = 1,
     .pad1 = 0,
     .name = "RxPDO assignment",
     .objdesc = kRxAssignment     },
    {        .index = TX_PDO_OBJIDX,
     .objtype = OTYPE_ARRAY,
     .maxsub = 1,
     .pad1 = 0,
     .name = "TxPDO assignment",
     .objdesc = kTxAssignment     },
    {   .index = Dictionary::kInput,
     .objtype = OTYPE_RECORD,
     .maxsub = Dictionary::kEntries,
     .pad1 = 0,
     .name = "Inputs",
     .objdesc = kInputData.data() },
    {  .index = Dictionary::kOutput,
     .objtype = OTYPE_RECORD,
     .maxsub = Dictionary::kEntries,
     .pad1 = 0,
     .name = "Outputs",
     .objdesc = kOutputData.data()},
    {               .index = 0xffff,
     .objtype = 0xff,
     .maxsub = 0xff,
     .pad1 = 0,
     .name = nullptr,
     .objdesc = nullptr           },
};

} // namespace librmcs::firmware::ecat
