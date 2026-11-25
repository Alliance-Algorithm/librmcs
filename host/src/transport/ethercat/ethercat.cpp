#include "host/src/transport/ethercat/ethercat.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <exception>
#include <format>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <stop_token>
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
#include <sys/timerfd.h>
#include <unistd.h>

// clang-format off
#include <linux/if.h> // IWYU pragma: keep (IFF_LOWER_UP; must follow net/if.h)
// clang-format on

#include <librmcs/board/common.hpp>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/ethercat/process_data.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/transport/transport.hpp"
#include "host/src/utility/ring_buffer.hpp"

namespace librmcs::host::transport::ethercat {

using core::protocol::ethercat::ProcessData;
using core::protocol::ethercat::ProcessDataChannel;

class Master : private core::utility::Immovable {
public:
    using FrameBuffer = std::array<std::byte, EC_BUFSIZE>;
    // Leave SOEM's index range available for startup and shutdown datagrams.
    static constexpr uint8_t kFirstCyclicIndex = EC_MAXBUF;

    struct CycleReply {
        uint8_t index;
        int wkc;
        std::span<const std::byte> input;
        std::optional<ec_alstatust> al_status;
    };

    void start(std::string_view interface, bool allow_version_mismatch) {
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

    ~Master() { close(); }

    void close() noexcept {
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

    ProcessData::Output outputs() {
        return std::as_writable_bytes(std::span{port_.txbuf[kOutputFrame]})
            .subspan<kPayloadOffset, ProcessData::kSize>();
    }

    void send_cycle(uint8_t index) {
        port_.txbuf[kOutputFrame][kIndexOffset] = index;
        port_.txbuf[kInputFrame][kIndexOffset] = index;
        port_.txbuf[kInputFrame][kAlIndexOffset] = index;
        if (try_send_frame(kOutputFrame))
            static_cast<void>(try_send_frame(kInputFrame));
    }

    std::optional<CycleReply> receive_cycle(FrameBuffer& buffer) const {
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
            reply.al_status = al;
        }
        return reply;
    }

    int expected_wkc() const { return expected_wkc_; }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr uint16_t kSlave = 1;
    static constexpr size_t kOutputFrame = 0;
    static constexpr size_t kInputFrame = 1;
    static constexpr unsigned kDatagramLengthMask = 0x07ffU;
    static constexpr unsigned kCirculating = 0x4000U;
    static constexpr size_t kIndexOffset = ETH_HEADERSIZE + offsetof(ec_comt, index);
    static constexpr size_t kPayloadOffset = ETH_HEADERSIZE + EC_HEADERSIZE;
    static constexpr size_t kPdoFrameSize = kPayloadOffset + ProcessData::kSize + EC_WKCSIZE;
    static constexpr size_t kAlIndexOffset =
        kPdoFrameSize - EC_ELENGTHSIZE + offsetof(ec_comt, index);
    static constexpr size_t kAlPayloadOffset = kPdoFrameSize - EC_ELENGTHSIZE + EC_HEADERSIZE;
    static constexpr size_t kAlFrameSize = kAlPayloadOffset + sizeof(ec_alstatust) + EC_WKCSIZE;
    static_assert(kAlFrameSize <= EC_BUFSIZE);

    void exchange() {
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRET);
    }

    bool state_is(uint16_t state) {
        ecx_statecheck(&context_, kSlave, state, EC_TIMEOUTRET);
        return slaves_[kSlave].state == state;
    }

    void prepare_cyclic_frames() {
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

    bool try_send_frame(size_t frame) {
        const auto size = static_cast<size_t>(port_.txbuflength[frame]);
        const auto sent = ::send(port_.sockhandle, port_.txbuf[frame], size, MSG_DONTWAIT);
        if (sent >= 0)
            return std::cmp_equal(sent, size);
        if (errno == EAGAIN || errno == ENOBUFS || errno == EINTR)
            return false;
        throw std::system_error{errno, std::generic_category(), "Sending EtherCAT PDO"};
    }

    static uint16_t read_word(const FrameBuffer& buffer, size_t offset) {
        uint16_t word{};
        std::memcpy(&word, buffer.data() + offset, sizeof(word));
        return etohs(word);
    }

    static void check_interface_link(const std::string& name) {
        ifaddrs* addresses = nullptr;
        if (getifaddrs(&addresses) != 0)
            return;
        const std::unique_ptr<ifaddrs, decltype(&freeifaddrs)> interfaces{addresses, freeifaddrs};
        for (const auto* entry = interfaces.get(); entry; entry = entry->ifa_next) {
            if (name != entry->ifa_name)
                continue;
            if ((entry->ifa_flags & IFF_UP) == 0)
                throw std::runtime_error{
                    "EtherCAT interface " + name + " is administratively down"};
            if ((entry->ifa_flags & IFF_LOWER_UP) == 0)
                throw std::runtime_error{"EtherCAT interface " + name + " has no link carrier"};
            return;
        }
    }

    void check_version(bool allow_mismatch) {
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
            throw std::runtime_error{std::format(
                "EtherCAT firmware version mismatch: {} (expected {})", actual, expected)};
        }
        logging::get_logger().warn(
            "EtherCAT firmware version mismatch: {} (expected {}); version override enabled",
            actual, expected);
    }

    void configure_process_data() {
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
        if (expected_wkc_ != 3)
            throw std::runtime_error{"Invalid EtherCAT working counter configuration"};
    }

    void require_state(uint16_t state) {
        ecx_statecheck(&context_, kSlave, state, EC_TIMEOUTSTATE);
        if (slaves_[kSlave].state != state)
            fail_state(state);
    }

    [[noreturn]] void fail_state(uint16_t requested) const {
        throw std::runtime_error{std::format(
            "EtherCAT state {:#04x}, AL status code {:#06x} (requested {:#04x})",
            slaves_[kSlave].state, slaves_[kSlave].ALstatuscode, requested)};
    }

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
    int expected_wkc_ = 0;
    bool socket_initialized_ = false;
    bool selected_ = false;
};

class Ethercat final : public Transport {
public:
    Ethercat(const board::EthercatOptions& ethercat, const board::AdvancedOptions& options) {
        for (size_t i = 0; i < kBufferCount; ++i)
            core::utility::assert_always(free_.push_back(std::make_unique<Buffer>(*this)));

        std::random_device entropy;
        process_data_.reset(std::uniform_int_distribution<uint32_t>{1, UINT32_MAX}(entropy));
        master_.start(ethercat.interface, options.dangerously_skip_version_checks);
        process_data_.write(master_.outputs());
        connected_.store(true, std::memory_order_relaxed);

        std::promise<void> setup;
        const auto setup_done = setup.get_future();
        thread_ = std::jthread{
            [this, &options, setup = std::move(setup)](const std::stop_token& stop_token) mutable {
                if (options.thread_setup)
                    options.thread_setup(options);
                setup.set_value();
                run(stop_token);
            }};
        setup_done.wait();
    }

    ~Ethercat() override {
        stop();
        if (thread_.joinable())
            thread_.join();
    }

    void stop() noexcept override {
        connected_.store(false, std::memory_order_relaxed);
        thread_.request_stop();
    }

    bool connected() const noexcept override { return connected_.load(std::memory_order_relaxed); }

    std::unique_ptr<TransportBuffer> acquire_transmit_buffer(TransmitKind kind) noexcept override {
        const std::scoped_lock lock{buffer_mutex_};
        if (!connected())
            return nullptr;
        if (kind == TransmitKind::kSessionControl)
            return std::move(free_session_);
        std::unique_ptr<TransportBuffer> buffer;
        free_.pop_front([&buffer](std::unique_ptr<TransportBuffer>&& available) noexcept {
            buffer = std::move(available);
        });
        return buffer;
    }

    void transmit(std::unique_ptr<TransportBuffer> buffer, size_t size) override {
        verify_buffer(buffer.get());
        core::utility::assert_always(size != 0 && size <= buffer->data().size());
        const std::scoped_lock lock{buffer_mutex_};
        if (!connected()) {
            recycle_buffer(std::move(buffer));
        } else if (buffer.get() == session_buffer_) {
            core::utility::assert_always(!pending_session_.buffer);
            pending_session_ = {.buffer = std::move(buffer), .size = size};
        } else {
            core::utility::assert_always(pending_.emplace_back(std::move(buffer), size));
        }
    }

    void release_transmit_buffer(std::unique_ptr<TransportBuffer> buffer) override {
        verify_buffer(buffer.get());
        const std::scoped_lock lock{buffer_mutex_};
        recycle_buffer(std::move(buffer));
    }

    void receive(std::function<void(std::span<const std::byte>)> callback) override {
        core::utility::assert_always(callback && !receiving_.load(std::memory_order_relaxed));
        callback_ = std::move(callback);
        receiving_.store(true, std::memory_order_release);
    }

private:
    using Clock = std::chrono::steady_clock;

    class Timer : private core::utility::Immovable {
    public:
        Timer()
            : fd_(::timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC)) {
            if (fd_ < 0)
                throw std::system_error{errno, std::generic_category(), "Creating EtherCAT timer"};
        }

        ~Timer() { ::close(fd_); }

        void arm(const itimerspec& schedule) const {
            if (::timerfd_settime(fd_, 0, &schedule, nullptr) < 0)
                throw std::system_error{errno, std::generic_category(), "Arming EtherCAT timer"};
        }

        void consume() const {
            uint64_t expirations{};
            while (::read(fd_, &expirations, sizeof(expirations)) < 0) {
                if (errno != EINTR)
                    throw std::system_error{
                        errno, std::generic_category(), "Reading EtherCAT timer"};
            }
        }

    private:
        int fd_;
    };

    class Buffer final : public TransportBuffer {
    public:
        explicit Buffer(const Ethercat& owner)
            : owner_(owner) {}
        BufferSpanType data() const noexcept override { return data_; }
        const Ethercat& owner() const { return owner_; }

    private:
        const Ethercat& owner_;
        mutable std::array<std::byte, core::protocol::kProtocolBufferSize> data_{};
    };

    struct Transfer {
        std::unique_ptr<TransportBuffer> buffer;
        size_t size;
    };

    void verify_buffer(const TransportBuffer* buffer) const {
        const auto* own = dynamic_cast<const Buffer*>(buffer);
        core::utility::assert_always(own && &own->owner() == this);
    }

    void recycle_buffer(std::unique_ptr<TransportBuffer> buffer) noexcept {
        if (buffer.get() == session_buffer_) {
            core::utility::assert_always(!free_session_);
            free_session_ = std::move(buffer);
        } else {
            core::utility::assert_always(free_.push_back(std::move(buffer)));
        }
    }

    void load_output() {
        if (process_data_.busy())
            return;
        const std::unique_lock lock{buffer_mutex_, std::try_to_lock};
        if (!lock.owns_lock())
            return;
        const auto enqueue = [this](Transfer transfer) noexcept {
            core::utility::assert_always(
                process_data_.enqueue(transfer.buffer->data().first(transfer.size)));
            recycle_buffer(std::move(transfer.buffer));
        };
        if (!pending_session_.buffer) {
            pending_.pop_front(enqueue);
            return;
        }

        // Carry session control with the next data packet when both fit in one PDO.
        auto session = std::exchange(pending_session_, {});
        if (const auto* next = pending_.peek_front();
            next && next->size <= session.buffer->data().size() - session.size) {
            pending_.pop_front([this, &session](Transfer data) noexcept {
                const auto destination = session.buffer->data().subspan(session.size, data.size);
                std::ranges::copy(data.buffer->data().first(data.size), destination.begin());
                session.size += data.size;
                recycle_buffer(std::move(data.buffer));
            });
        }
        enqueue(std::move(session));
    }

    bool process_input(std::span<const std::byte> input) {
        const auto frame = ProcessData::decode(input);
        if (!frame)
            return false;
        const auto received = [&] {
            const std::scoped_lock lock{process_data_mutex_};
            return process_data_.receive(*frame);
        }();
        if (!received)
            return false;
        if (!received->empty() && receiving_.load(std::memory_order_acquire))
            callback_(*received);
        return true;
    }

    void send_cycles(const std::stop_token& stop_token) {
        const Timer timer;
        constexpr std::timespec period{.tv_sec = 0, .tv_nsec = 1'000'000};
        timer.arm({.it_interval = period, .it_value = period});

        uint8_t index = Master::kFirstCyclicIndex;
        while (!stop_token.stop_requested() && connected()) {
            timer.consume();
            if (stop_token.stop_requested() || !connected())
                return;

            {
                // Do not block a transmit tick on receive-side acknowledgement processing.
                const std::unique_lock lock{process_data_mutex_, std::try_to_lock};
                if (lock.owns_lock()) {
                    load_output();
                    process_data_.write(master_.outputs());
                }
            }
            // The input PDO carries the latest published data, including cumulative ACKs.
            master_.send_cycle(index);
            index =
                index == UINT8_MAX ? Master::kFirstCyclicIndex : static_cast<uint8_t>(index + 1);
        }
    }

    void receive_cycles(const std::stop_token& stop_token) {
        constexpr auto timeout = std::chrono::milliseconds{100};
        auto last_response = Clock::now();
        Master::FrameBuffer buffer;
        std::optional<uint8_t> output_index;
        int output_wkc = 0;
        int wkc = EC_NOFRAME;
        while (!stop_token.stop_requested() && connected()) {
            const auto reply = master_.receive_cycle(buffer);
            const auto now = Clock::now();
            if (reply && reply->input.empty()) {
                output_index = reply->index;
                output_wkc = reply->wkc;
            } else if (reply) {
                if (reply->al_status && etohs(reply->al_status->alstatus) != EC_STATE_OPERATIONAL) {
                    logging::get_logger().warn(
                        "EtherCAT Ultra left OP (state {:#04x}, AL status code {:#06x})",
                        etohs(reply->al_status->alstatus), etohs(reply->al_status->alstatuscode));
                    return;
                }
                const bool paired = output_index == reply->index;
                output_index.reset();
                wkc = reply->wkc + (paired ? output_wkc : 0);
                if (paired && output_wkc == 2 && reply->wkc == 1 && process_input(reply->input))
                    last_response = now;
            } else {
                wkc = EC_NOFRAME;
            }
            if (now - last_response >= timeout) {
                logging::get_logger().warn(
                    "EtherCAT process data lost (WKC {}, expected {})", wkc,
                    master_.expected_wkc());
                return;
            }
        }
    }

    void run(const std::stop_token& stop_token) noexcept {
        try {
            const std::jthread sender{[this](const std::stop_token& sender_stop) {
                try {
                    send_cycles(sender_stop);
                } catch (const std::exception& e) {
                    connected_.store(false, std::memory_order_relaxed);
                    logging::get_logger().error("EtherCAT PDO sender stopped: {}", e.what());
                }
            }};
            receive_cycles(stop_token);
        } catch (const std::exception& e) {
            logging::get_logger().error("EtherCAT PDO receiver stopped: {}", e.what());
        }
        connected_.store(false, std::memory_order_relaxed);
        master_.close();
    }

    static constexpr size_t kBufferCount = 16;
    Master master_;
    ProcessDataChannel<2> process_data_;
    std::mutex process_data_mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> receiving_{false};
    std::mutex buffer_mutex_;
    utility::RingBuffer<std::unique_ptr<TransportBuffer>> free_{kBufferCount};
    utility::RingBuffer<Transfer> pending_{kBufferCount};
    // Application traffic must leave enough capacity to refresh the session.
    std::unique_ptr<TransportBuffer> free_session_{std::make_unique<Buffer>(*this)};
    const TransportBuffer* const session_buffer_ = free_session_.get();
    Transfer pending_session_{};
    std::function<void(std::span<const std::byte>)> callback_;
    std::jthread thread_;
};

std::unique_ptr<Transport> EthercatFactory::create(
    const board::EthercatOptions& ethercat, const board::AdvancedOptions& options) {
    return std::make_unique<Ethercat>(ethercat, options);
}

} // namespace librmcs::host::transport::ethercat
