#include "host/src/transport/ethercat/ethercat.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
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
#include <ethercatcoe.h>
#include <ethercatconfig.h>
#include <ethercatmain.h>
#include <ethercattype.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <nicdrv.h>
#include <osal.h>
#include <pthread.h>

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

class Master : private core::utility::Immovable {
public:
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
        return std::as_writable_bytes(
            std::span<uint8_t, ProcessData::kSize>{slaves_[kSlave].outputs, ProcessData::kSize});
    }

    std::span<const std::byte> inputs() const {
        return std::as_bytes(std::span{slaves_[kSlave].inputs, ProcessData::kSize});
    }

    int exchange() {
        ecx_send_processdata(&context_);
        return ecx_receive_processdata(&context_, EC_TIMEOUTRET);
    }

    int expected_wkc() const { return expected_wkc_; }

    bool state_is(uint16_t state) {
        ecx_statecheck(&context_, kSlave, state, EC_TIMEOUTRET);
        return slaves_[kSlave].state == state;
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr uint16_t kSlave = 1;

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
            || device.outputs != io_map_.data()
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

    std::unique_ptr<TransportBuffer> acquire_transmit_buffer() noexcept override {
        const std::scoped_lock lock{buffer_mutex_};
        if (!connected())
            return nullptr;
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
        if (connected()) {
            core::utility::assert_always(pending_.emplace_back(std::move(buffer), size));
        } else {
            core::utility::assert_always(free_.push_back(std::move(buffer)));
        }
    }

    void release_transmit_buffer(std::unique_ptr<TransportBuffer> buffer) override {
        verify_buffer(buffer.get());
        const std::scoped_lock lock{buffer_mutex_};
        core::utility::assert_always(free_.push_back(std::move(buffer)));
    }

    void receive(std::function<void(std::span<const std::byte>)> callback) override {
        core::utility::assert_always(callback && !receiving_.load(std::memory_order_relaxed));
        callback_ = std::move(callback);
        receiving_.store(true, std::memory_order_release);
    }

private:
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

    void load_output() {
        if (process_data_.busy())
            return;
        const std::scoped_lock lock{buffer_mutex_};
        pending_.pop_front([this](Transfer transfer) noexcept {
            core::utility::assert_always(
                process_data_.enqueue(transfer.buffer->data().first(transfer.size)));
            core::utility::assert_always(free_.push_back(std::move(transfer.buffer)));
        });
    }

    bool process_input() {
        const auto frame = ProcessData::decode(master_.inputs());
        if (!frame)
            return false;
        const auto received = process_data_.receive(*frame);
        if (received && !received->empty() && receiving_.load(std::memory_order_acquire))
            callback_(*received);
        return received.has_value();
    }

    void run(const std::stop_token& stop_token) noexcept {
        using Clock = std::chrono::steady_clock;
        constexpr auto cycle = std::chrono::milliseconds{2};
        constexpr auto timeout = std::chrono::milliseconds{100};
        auto last_response = Clock::now();
        auto last_state_check = last_response;
        try {
            while (!stop_token.stop_requested()) {
                const auto now = Clock::now();
                load_output();
                process_data_.write(master_.outputs());
                const int wkc = master_.exchange();
                if (wkc == master_.expected_wkc() && process_input())
                    last_response = now;
                if (now - last_response >= timeout) {
                    logging::get_logger().warn(
                        "EtherCAT process data lost (WKC {}, expected {})", wkc,
                        master_.expected_wkc());
                    break;
                }
                if (now - last_state_check >= timeout) {
                    last_state_check = now;
                    if (!master_.state_is(EC_STATE_OPERATIONAL)) {
                        logging::get_logger().warn("EtherCAT Ultra left OP");
                        break;
                    }
                }
                std::this_thread::sleep_until(now + cycle);
            }
        } catch (const std::exception& e) {
            logging::get_logger().error("EtherCAT event loop stopped: {}", e.what());
        }
        connected_.store(false, std::memory_order_relaxed);
        master_.close();
    }

    static constexpr size_t kBufferCount = 16;
    Master master_;
    ProcessData process_data_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> receiving_{false};
    std::mutex buffer_mutex_;
    utility::RingBuffer<std::unique_ptr<TransportBuffer>> free_{kBufferCount};
    utility::RingBuffer<Transfer> pending_{kBufferCount};
    std::function<void(std::span<const std::byte>)> callback_;
    std::jthread thread_;
};

std::unique_ptr<Transport> EthercatFactory::create(
    const board::EthercatOptions& ethercat, const board::AdvancedOptions& options) {
    return std::make_unique<Ethercat>(ethercat, options);
}

} // namespace librmcs::host::transport::ethercat
