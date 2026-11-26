#include "host/src/transport/ethercat/ethercat.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <span>
#include <stop_token>
#include <system_error>
#include <thread>
#include <utility>

#include <sys/timerfd.h>
#include <unistd.h>

#include <librmcs/board/common.hpp>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/ethercat/process_data.hpp"
#include "core/src/protocol/ethercat/process_data_channel.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/transport/ethercat/master.hpp"
#include "host/src/transport/transport.hpp"
#include "host/src/utility/ring_buffer.hpp"

namespace librmcs::host::transport::ethercat {

using core::protocol::ethercat::ProcessData;
using core::protocol::ethercat::ProcessDataChannel;

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
            master_.send_cycle();
        }
    }

    void receive_cycles(const std::stop_token& stop_token) {
        constexpr auto timeout = std::chrono::milliseconds{100};
        auto last_response = Clock::now();
        std::optional<uint8_t> output_index;
        int output_wkc = 0;
        int wkc = Master::kNoFrame;
        while (!stop_token.stop_requested() && connected()) {
            const auto reply = master_.receive_cycle();
            const auto now = Clock::now();
            if (reply && reply->input.empty()) {
                output_index = reply->index;
                output_wkc = reply->wkc;
            } else if (reply) {
                if (reply->al_status && !reply->al_status->is_operational()) {
                    logging::get_logger().warn(
                        "EtherCAT Ultra left OP (state {:#04x}, AL status code {:#06x})",
                        reply->al_status->state, reply->al_status->code);
                    return;
                }
                const bool paired = output_index == reply->index;
                output_index.reset();
                wkc = reply->wkc + (paired ? output_wkc : 0);
                if (paired && output_wkc == Master::kOutputWkc && reply->wkc == Master::kInputWkc
                    && process_input(reply->input))
                    last_response = now;
            } else {
                wkc = Master::kNoFrame;
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
