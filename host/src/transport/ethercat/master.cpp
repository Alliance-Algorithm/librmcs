#include "host/src/transport/ethercat/master.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

#include <ethercat.h> // IWYU pragma: keep
#include <ethercatbase.h>
#include <ethercatcoe.h>
#include <ethercatconfig.h>
#include <ethercatmain.h>
#include <ethercattype.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <nicdrv.h>
#include <osal.h>
#include <pthread.h>
#include <sys/poll.h>
#include <sys/socket.h>

// clang-format off
#include <linux/if.h> // IWYU pragma: keep (IFF_LOWER_UP; must follow net/if.h)
// clang-format on

#include "core/src/protocol/ethercat/process_data.hpp"
#include "host/src/logging/logging.hpp"

namespace librmcs::host::transport::ethercat {

using core::protocol::ethercat::ProcessData;

void Master::start(std::string_view interface, bool allow_version_mismatch) {
    using Clock = std::chrono::steady_clock;

    if (interface.empty() || interface.size() >= IFNAMSIZ || interface.contains('\0')) {
        throw std::invalid_argument{"Invalid EtherCAT interface name"};
    }
    const std::string name{interface};
    if (!if_nametoindex(name.c_str()))
        throw std::system_error{errno, std::generic_category(), "EtherCAT interface " + name};

    socket_initialized_ = true;
    if (ecx_init(&context_, name.c_str()) <= 0) {
        throw std::system_error{
            errno, std::generic_category(), "Opening raw EtherCAT socket on " + name};
    }

    constexpr size_t slave = kSlave;
    const auto count = ecx_config_init(&context_, FALSE);
    if (count <= 0)
        check_interface_link(name);
    if (count != 1)
        throw std::runtime_error{std::format("Expected one EtherCAT slave, found {}", count)};

    using core::protocol::ethercat::kProductCode;
    using core::protocol::ethercat::kRevision;
    using core::protocol::ethercat::kVendorId;
    if (slaves_[slave].eep_man != kVendorId || slaves_[slave].eep_id != kProductCode
        || slaves_[slave].eep_rev != kRevision) {
        throw std::runtime_error{std::format(
            "Unexpected EtherCAT identity (vendor {:#x}, product {:#x}, revision {})",
            slaves_[slave].eep_man, slaves_[slave].eep_id, slaves_[slave].eep_rev)};
    }

    selected_ = true;
    require_state(EC_STATE_PRE_OP);
    check_version(allow_version_mismatch);
    configure_process_data();
    require_state(EC_STATE_SAFE_OP);

    exchange(); // Prime SM2 before the OP transition.
    slaves_[slave].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&context_, kSlave);
    const auto deadline = Clock::now() + std::chrono::seconds{1};
    do {
        exchange();
        if (state_is(EC_STATE_OPERATIONAL)) {
            prepare_cyclic_frames();
            logging::get_logger().info(
                "EtherCAT Ultra is OP on {} ({} bytes each direction, expected WKC {})", name,
                ProcessData::kSize, expected_wkc_);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    } while (Clock::now() < deadline && (slaves_[slave].state & EC_STATE_ERROR) == 0);
    fail_state(EC_STATE_OPERATIONAL);
}

Master::~Master() { close(); }

void Master::close() noexcept {
    if (!socket_initialized_)
        return;
    if (selected_) {
        slaves_[kSlave].state = EC_STATE_INIT;
        ecx_writestate(&context_, kSlave);
    }
    ecx_close(&context_);
    pthread_mutex_destroy(&port_.rx_mutex);
    pthread_mutex_destroy(&port_.tx_mutex);
    pthread_mutex_destroy(&port_.getindex_mutex);
    socket_initialized_ = false;
}

ProcessData::Output Master::outputs() {
    return std::as_writable_bytes(std::span{port_.txbuf[kOutputFrame]})
        .subspan<kPayloadOffset, ProcessData::kSize>();
}

void Master::send_cycle() {
    const auto index = next_cyclic_index_;
    port_.txbuf[kOutputFrame][kIndexOffset] = index;
    port_.txbuf[kInputFrame][kIndexOffset] = index;
    port_.txbuf[kInputFrame][kAlIndexOffset] = index;
    if (try_send_frame(kOutputFrame))
        static_cast<void>(try_send_frame(kInputFrame));
    next_cyclic_index_ = index == UINT8_MAX ? kFirstCyclicIndex : static_cast<uint8_t>(index + 1);
}

std::optional<Master::CycleReply> Master::receive_cycle() {
    pollfd socket{.fd = port_.sockhandle, .events = POLLIN, .revents = 0};
    constexpr int receive_wait_ms = 10;
    const int ready = ::poll(&socket, 1, receive_wait_ms);
    if (ready < 0 && errno != EINTR)
        throw std::system_error{errno, std::generic_category(), "Polling EtherCAT PDO socket"};
    if (ready <= 0)
        return std::nullopt;
    constexpr unsigned error_events = POLLERR | POLLHUP | POLLNVAL;
    if ((static_cast<unsigned>(socket.revents) & error_events) != 0)
        throw std::runtime_error{"EtherCAT PDO socket became unavailable"};

    auto& buffer = receive_buffer_;
    const auto size = ::recv(socket.fd, buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (size < 0 && errno != EAGAIN && errno != EINTR)
        throw std::system_error{errno, std::generic_category(), "Receiving EtherCAT PDO"};
    if (std::cmp_less(size, kPdoFrameSize))
        return std::nullopt;

    ec_comt header{};
    std::memcpy(&header, buffer.data() + ETH_HEADERSIZE, sizeof(header));
    const auto length = static_cast<unsigned>(etohs(header.dlength));
    if (header.command != EC_CMD_LRW || header.index < kFirstCyclicIndex
        || (length & kDatagramLengthMask) != ProcessData::kSize || (length & kCirculating) == 0)
        return std::nullopt;

    const auto logical_address = (static_cast<uint32_t>(etohs(header.ADO)) << 16U)
                               | static_cast<uint32_t>(etohs(header.ADP));
    CycleReply reply{
        .index = header.index,
        .wkc = read_word(buffer, kPdoFrameSize - EC_WKCSIZE),
        .input = {},
        .al_status = std::nullopt,
    };
    if (logical_address == groups_[0].logstartaddr)
        return reply;
    if (logical_address != groups_[0].logstartaddr + ProcessData::kSize
        || (length & static_cast<unsigned>(EC_DATAGRAMFOLLOWS)) == 0
        || std::cmp_less(size, kAlFrameSize))
        return std::nullopt;

    reply.input = std::span{buffer}.subspan(kPayloadOffset, ProcessData::kSize);
    ec_comt al_header{};
    std::memcpy(&al_header, buffer.data() + kPdoFrameSize - EC_ELENGTHSIZE, sizeof(al_header));
    if (al_header.command == EC_CMD_FPRD && al_header.index == header.index
        && etohs(al_header.ADP) == slaves_[kSlave].configadr
        && etohs(al_header.ADO) == ECT_REG_ALSTAT
        && (etohs(al_header.dlength) & kDatagramLengthMask) == sizeof(ec_alstatust)
        && read_word(buffer, kAlFrameSize - EC_WKCSIZE) == 1) {
        ec_alstatust al{};
        std::memcpy(&al, buffer.data() + kAlPayloadOffset, sizeof(al));
        reply.al_status = AlStatus{.state = etohs(al.alstatus), .code = etohs(al.alstatuscode)};
    }
    return reply;
}

void Master::exchange() {
    ecx_send_processdata(&context_);
    ecx_receive_processdata(&context_, EC_TIMEOUTRET);
}

bool Master::state_is(uint16_t state) {
    ecx_statecheck(&context_, kSlave, state, EC_TIMEOUTRET);
    return slaves_[kSlave].state == state;
}

void Master::prepare_cyclic_frames() {
    // Split I/O uses the same logical mapping as the startup exchange.
    // The receiver reads the socket directly and leaves SOEM's index stack alone.
    std::array<std::byte, ProcessData::kSize> empty_pdo{};
    const auto output_address = groups_[0].logstartaddr;
    const auto input_address = output_address + static_cast<uint32_t>(ProcessData::kSize);
    ecx_setupdatagram(
        &port_, port_.txbuf[kOutputFrame], EC_CMD_LRW, kOutputFrame,
        static_cast<uint16_t>(output_address), static_cast<uint16_t>(output_address >> 16U),
        ProcessData::kSize, empty_pdo.data());
    ecx_setupdatagram(
        &port_, port_.txbuf[kInputFrame], EC_CMD_LRW, kInputFrame,
        static_cast<uint16_t>(input_address), static_cast<uint16_t>(input_address >> 16U),
        ProcessData::kSize, empty_pdo.data());
    ecx_adddatagram(
        &port_, port_.txbuf[kInputFrame], EC_CMD_FPRD, kInputFrame, FALSE,
        slaves_[kSlave].configadr, ECT_REG_ALSTAT, sizeof(ec_alstatust), nullptr);
}

bool Master::try_send_frame(size_t frame) {
    const auto size = static_cast<size_t>(port_.txbuflength[frame]);
    const auto sent = ::send(port_.sockhandle, port_.txbuf[frame], size, MSG_DONTWAIT);
    if (sent >= 0)
        return std::cmp_equal(sent, size);
    if (errno == EAGAIN || errno == ENOBUFS || errno == EINTR)
        return false;
    throw std::system_error{errno, std::generic_category(), "Sending EtherCAT PDO"};
}

uint16_t Master::read_word(const FrameBuffer& buffer, size_t offset) {
    uint16_t word{};
    std::memcpy(&word, buffer.data() + offset, sizeof(word));
    return etohs(word);
}

void Master::check_interface_link(const std::string& name) {
    ifaddrs* addresses = nullptr;
    if (getifaddrs(&addresses) != 0)
        return;
    const std::unique_ptr<ifaddrs, decltype(&freeifaddrs)> interfaces{addresses, freeifaddrs};
    for (const auto* entry = interfaces.get(); entry; entry = entry->ifa_next) {
        if (name != entry->ifa_name)
            continue;
        if ((entry->ifa_flags & IFF_UP) == 0)
            throw std::runtime_error{"EtherCAT interface " + name + " is administratively down"};
        if ((entry->ifa_flags & IFF_LOWER_UP) == 0)
            throw std::runtime_error{"EtherCAT interface " + name + " has no link carrier"};
        return;
    }
}

void Master::check_version(bool allow_mismatch) {
    std::array<char, 128> version{};
    auto size = static_cast<int>(version.size());
    if (ecx_SDOread(
            &context_, kSlave, core::protocol::ethercat::kSoftwareVersionIndex, 0, FALSE, &size,
            version.data(), EC_TIMEOUTRXM)
            <= 0
        || size <= 0 || std::cmp_greater(size, version.size())) {
        throw std::runtime_error{"Could not read Ultra firmware version from EtherCAT CoE"};
    }

    const std::string_view actual{version.data(), static_cast<size_t>(size)};
    constexpr std::string_view expected = LIBRMCS_PROJECT_VERSION_STRING;
    if (actual == expected)
        return;
    if (!allow_mismatch) {
        throw std::runtime_error{
            std::format("EtherCAT firmware version mismatch: {} (expected {})", actual, expected)};
    }
    logging::get_logger().warn(
        "EtherCAT firmware version mismatch: {} (expected {}); version override enabled", actual,
        expected);
}

void Master::configure_process_data() {
    // SOEM uses shared mapping worker state even with separate master contexts.
    const std::scoped_lock lock{mapping_mutex_};
    const int mapped = ecx_config_map_group(&context_, io_map_.data(), 0);
    const auto& device = slaves_[kSlave];
    if (mapped != 2 * ProcessData::kSize || device.Obytes != ProcessData::kSize
        || device.Ibytes != ProcessData::kSize || device.Ostartbit != 0 || device.Istartbit != 0
        || groups_[0].blockLRW != 0 || device.outputs != io_map_.data()
        || device.inputs != io_map_.data() + ProcessData::kSize) {
        throw std::runtime_error{std::format(
            "Unexpected EtherCAT PDO mapping (out {}, in {}, total {})", device.Obytes,
            device.Ibytes, mapped)};
    }
    expected_wkc_ = (2 * groups_[0].outputsWKC) + groups_[0].inputsWKC;
    if (expected_wkc_ != kOutputWkc + kInputWkc)
        throw std::runtime_error{"Invalid EtherCAT working counter configuration"};
}

void Master::require_state(uint16_t state) {
    ecx_statecheck(&context_, kSlave, state, EC_TIMEOUTSTATE);
    if (slaves_[kSlave].state != state)
        fail_state(state);
}

void Master::fail_state(uint16_t requested) const {
    throw std::runtime_error{std::format(
        "EtherCAT state {:#04x}, AL status code {:#06x} (requested {:#04x})", slaves_[kSlave].state,
        slaves_[kSlave].ALstatuscode, requested)};
}

} // namespace librmcs::host::transport::ethercat
