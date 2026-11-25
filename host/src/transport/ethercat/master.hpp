#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include <ethercat.h> // IWYU pragma: keep
#include <ethercatmain.h>
#include <ethercattype.h>
#include <nicdrv.h>
#include <osal.h>

#include "core/src/protocol/ethercat/process_data.hpp"
#include "core/src/utility/immovable.hpp"

namespace librmcs::host::transport::ethercat {

class Master : private core::utility::Immovable {
public:
    static constexpr int kOutputWkc = 2;
    static constexpr int kInputWkc = 1;
    static constexpr int kNoFrame = EC_NOFRAME;

    struct AlStatus {
        uint16_t state;
        uint16_t code;

        bool is_operational() const { return state == EC_STATE_OPERATIONAL; }
    };

    struct CycleReply {
        uint8_t index;
        int wkc;
        std::span<const std::byte> input;
        std::optional<AlStatus> al_status;
    };

    ~Master();

    void start(std::string_view interface, bool allow_version_mismatch);
    void close() noexcept;

    core::protocol::ethercat::ProcessData::Output outputs();
    void send_cycle();
    // The returned input span is valid until the next receive.
    std::optional<CycleReply> receive_cycle();

    int expected_wkc() const { return expected_wkc_; }

private:
    using FrameBuffer = std::array<std::byte, EC_BUFSIZE>;

    // Leave SOEM's index range available for startup and shutdown datagrams.
    static constexpr uint8_t kFirstCyclicIndex = EC_MAXBUF;
    static constexpr uint16_t kSlave = 1;
    static constexpr size_t kOutputFrame = 0;
    static constexpr size_t kInputFrame = 1;
    static constexpr unsigned kDatagramLengthMask = 0x07ffU;
    static constexpr unsigned kCirculating = 0x4000U;
    static constexpr size_t kIndexOffset = ETH_HEADERSIZE + offsetof(ec_comt, index);
    static constexpr size_t kPayloadOffset = ETH_HEADERSIZE + EC_HEADERSIZE;
    static constexpr size_t kPdoFrameSize =
        kPayloadOffset + core::protocol::ethercat::ProcessData::kSize + EC_WKCSIZE;
    static constexpr size_t kAlIndexOffset =
        kPdoFrameSize - EC_ELENGTHSIZE + offsetof(ec_comt, index);
    static constexpr size_t kAlPayloadOffset = kPdoFrameSize - EC_ELENGTHSIZE + EC_HEADERSIZE;
    static constexpr size_t kAlFrameSize = kAlPayloadOffset + sizeof(ec_alstatust) + EC_WKCSIZE;
    static_assert(kAlFrameSize <= EC_BUFSIZE);

    void exchange();
    bool state_is(uint16_t state);
    void prepare_cyclic_frames();
    bool try_send_frame(size_t frame);
    static uint16_t read_word(const FrameBuffer& buffer, size_t offset);
    static void check_interface_link(const std::string& name);
    void check_version(bool allow_mismatch);
    void configure_process_data();
    void require_state(uint16_t state);
    [[noreturn]] void fail_state(uint16_t requested) const;

    inline static std::mutex mapping_mutex_;
    ecx_portt port_{};
    std::array<ec_slavet, EC_MAXSLAVE> slaves_{};
    int slave_count_ = 0;
    std::array<ec_groupt, EC_MAXGROUP> groups_{};
    std::array<uint8_t, EC_MAXEEPBUF> eeprom_cache_{};
    std::array<uint32_t, EC_MAXEEPBITMAP> eeprom_bitmap_{};
    ec_eringt errors_{};
    ec_idxstackT indices_{};
    boolean has_error_ = FALSE;
    int64 dc_time_ = 0;
    std::array<ec_SMcommtypet, EC_MAX_MAPT> sm_types_{};
    std::array<ec_PDOassignt, EC_MAX_MAPT> pdo_assignments_{};
    std::array<ec_PDOdesct, EC_MAX_MAPT> pdo_details_{};
    ec_eepromSMt sii_sm_{};
    ec_eepromFMMUt sii_fmmu_{};
    ecx_contextt context_{
        .port = &port_,
        .slavelist = slaves_.data(),
        .slavecount = &slave_count_,
        .maxslave = EC_MAXSLAVE,
        .grouplist = groups_.data(),
        .maxgroup = EC_MAXGROUP,
        .esibuf = eeprom_cache_.data(),
        .esimap = eeprom_bitmap_.data(),
        .esislave = 0,
        .elist = &errors_,
        .idxstack = &indices_,
        .ecaterror = &has_error_,
        .DCtO = 0,
        .DCl = 0,
        .DCtime = &dc_time_,
        .SMcommtype = sm_types_.data(),
        .PDOassign = pdo_assignments_.data(),
        .PDOdesc = pdo_details_.data(),
        .eepSM = &sii_sm_,
        .eepFMMU = &sii_fmmu_,
        .FOEhook = nullptr,
        .EOEhook = nullptr,
        .manualstatechange = 0,
    };
    // SOEM maps before reporting sizes. Reserve the full uint16_t bit count per direction.
    std::array<uint8_t, 2 * ((size_t{UINT16_MAX} + 1) / 8)> io_map_{};
    FrameBuffer receive_buffer_{};
    uint8_t next_cyclic_index_ = kFirstCyclicIndex;
    int expected_wkc_ = 0;
    bool socket_initialized_ = false;
    bool selected_ = false;
};

} // namespace librmcs::host::transport::ethercat
