#include "librmcs/protocol/handler.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <new>
#include <random>
#include <span>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/protocol/stream_buffer.hpp"
#include "host/src/transport/transport.hpp"
#include "librmcs/board/common.hpp"
#include "librmcs/data/datas.hpp"

namespace librmcs::host::protocol {

class Handler::Impl : public core::protocol::DeserializeCallback {
public:
    static constexpr auto kSessionAckTimeout = std::chrono::milliseconds{200};
    static constexpr size_t kSessionAckRetryCount = 5;
    static constexpr auto kSessionRefreshInterval = std::chrono::milliseconds{250};

    explicit Impl(std::unique_ptr<transport::Transport> transport, data::DataCallback& callback)
        : callback_(callback)
        , deserializer_(*this)
        , expected_session_nonce_(generate_session_nonce())
        , transport_(std::move(transport)) {
        transport_->receive([this](std::span<const std::byte> buffer) {
            // Operating system automatically assembles the packet
            deserializer_.feed(buffer);
            deserializer_.finish_transfer();
        });

        establish_session();
        keepalive_thread_ = std::thread{[this] { keepalive_loop(); }};
    }

    ~Impl() override {
        stop_keepalive_.store(true, std::memory_order_relaxed);
        session_cv_.notify_all();
        if (keepalive_thread_.joinable())
            keepalive_thread_.join();

        transport_.reset();
    }

    PacketBuilder start_transmit() { return PacketBuilder{transport_.get()}; }

    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!session_established())
            return true;
        if (!callback_.can_receive_callback(id, data)) {
            logging::get_logger().error("Unexpected can field id: ", static_cast<int>(id));
            return false;
        }
        return true;
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!session_established())
            return true;
        if (!callback_.uart_receive_callback(id, data)) {
            logging::get_logger().error("Unexpected uart field id: ", static_cast<int>(id));
            return false;
        }
        return true;
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (!session_established())
            return true;
        if (!callback_.gpio_digital_read_result_callback(channel_index, data)) {
            logging::get_logger().error(
                "Unexpected gpio channel index: ", static_cast<int>(channel_index));
            return false;
        }
        return true;
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (!session_established())
            return true;
        if (!callback_.gpio_analog_read_result_callback(channel_index, data)) {
            logging::get_logger().error(
                "Unexpected gpio channel index: ", static_cast<int>(channel_index));
            return false;
        }
        return true;
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established())
            return true;
        (void)channel_index;
        (void)data;
        logging::get_logger().error("Unexpected gpio digital read config field in uplink");
        return false;
    }

    bool gpio_analog_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (!session_established())
            return true;
        (void)channel_index;
        (void)data;
        logging::get_logger().error("Unexpected gpio analog read config field in uplink");
        return false;
    }

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override {
        if (!session_established())
            return;
        callback_.accelerometer_receive_callback(data);
    }

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override {
        if (!session_established())
            return;
        callback_.gyroscope_receive_callback(data);
    }

    void temperature_deserialized_callback(const data::TemperatureDataView& data) override {
        if (!session_established())
            return;
        callback_.temperature_receive_callback(data);
    }

    void session_control_deserialized_callback(const data::SessionControlView& data) override {
        if (data.nonce != expected_session_nonce_)
            return;

        bool notify = false;
        {
            const std::scoped_lock guard{session_mutex_};
            switch (data.type) {
            case data::SessionType::kStartAck:
                session_established_.store(true, std::memory_order_relaxed);
                ++session_start_ack_count_;
                notify = true;
                break;
            case data::SessionType::kKeepaliveAck:
                ++session_keepalive_ack_count_;
                notify = true;
                break;
            default: break;
            }
        }
        if (notify)
            session_cv_.notify_all();
    }

    void error_callback() override {
        logging::get_logger().error("Deserializer encountered an error while parsing input");
    }

private:
    [[nodiscard]] bool session_established() const {
        return session_established_.load(std::memory_order_relaxed);
    }

    void establish_session() {
        for (size_t attempt = 0; attempt < kSessionAckRetryCount; ++attempt) {
            uint64_t previous_session_start_ack_count = 0;
            {
                const std::scoped_lock guard{session_mutex_};
                previous_session_start_ack_count = session_start_ack_count_;
            }

            send_session_start();

            std::unique_lock lock{session_mutex_};
            if (session_cv_.wait_for(
                    lock, kSessionAckTimeout, [this, previous_session_start_ack_count] {
                        return session_start_ack_count_ > previous_session_start_ack_count;
                    })) {
                return;
            }
        }

        throw std::runtime_error{"Timed out waiting for SESSION_ACK"};
    }

    void send_session_start() { send_session_control(data::SessionType::kStart, "Session Start"); }

    void refresh_session() {
        for (size_t attempt = 0; attempt < kSessionAckRetryCount; ++attempt) {
            uint64_t previous_session_keepalive_ack_count = 0;
            {
                const std::scoped_lock guard{session_mutex_};
                previous_session_keepalive_ack_count = session_keepalive_ack_count_;
            }

            send_session_keepalive();

            std::unique_lock lock{session_mutex_};
            if (session_cv_.wait_for(
                    lock, kSessionAckTimeout, [this, previous_session_keepalive_ack_count] {
                        return stop_keepalive_.load(std::memory_order_relaxed)
                            || session_keepalive_ack_count_ > previous_session_keepalive_ack_count;
                    })) {
                return;
            }
        }

        throw std::runtime_error{"Timed out waiting for SESSION_KEEPALIVE_ACK"};
    }

    void send_session_keepalive() {
        send_session_control(data::SessionType::kKeepalive, "Session Keepalive");
    }

    void send_session_control(data::SessionType type, std::string_view operation_name) {
        core::protocol::Serializer::SerializeResult result;
        {
            StreamBuffer buffer{*transport_};
            core::protocol::Serializer serializer{buffer};
            result =
                serializer.write_session_control({.type = type, .nonce = expected_session_nonce_});
        }

        core::utility::assert_debug(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
        if (result == core::protocol::Serializer::SerializeResult::kBadAlloc) [[unlikely]]
            throw std::runtime_error(
                std::string{"Failed to transmit "} + std::string{operation_name}
                + ": Transmit buffer unavailable (acquire failed)");
    }

    void keepalive_loop() {
        while (!stop_keepalive_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(kSessionRefreshInterval);
            if (stop_keepalive_.load(std::memory_order_relaxed))
                break;

            try {
                refresh_session();
            } catch (const std::exception& exception) {
                logging::get_logger().error(
                    "Failed to refresh session: {}. Terminating...", exception.what());
                std::terminate();
            }
        }
    }

    static uint32_t generate_session_nonce() {
        std::random_device random_device;
        std::uniform_int_distribution<uint32_t> distribution;
        return distribution(random_device);
    }

    data::DataCallback& callback_;
    core::protocol::Deserializer deserializer_;

    mutable std::mutex session_mutex_;
    std::condition_variable session_cv_;
    std::atomic<bool> session_established_{false};
    uint64_t session_start_ack_count_ = 0;
    uint64_t session_keepalive_ack_count_ = 0;
    uint32_t expected_session_nonce_ = 0;

    std::unique_ptr<transport::Transport> transport_;

    std::atomic<bool> stop_keepalive_{false};
    std::thread keepalive_thread_;
};

namespace {

struct PacketBuilderImpl {
    explicit PacketBuilderImpl(transport::Transport& transport) noexcept
        : buffer_(transport)
        , serializer_(buffer_) {}

    PacketBuilderImpl(PacketBuilderImpl&& other) noexcept
        : buffer_(std::move(other.buffer_))
        , serializer_(buffer_) {}

    PacketBuilderImpl& operator=(PacketBuilderImpl&&) = delete;
    PacketBuilderImpl(const PacketBuilderImpl&) = delete;
    PacketBuilderImpl& operator=(const PacketBuilderImpl&) = delete;
    ~PacketBuilderImpl() = default;

    // `write_*` returns `true` if args are valid; it never reports transport/resource issues.
    // - `kInvalidArgument` => `false` (user error)
    // - `kBadAlloc` => logged and ignored (`true`) (internal/transient)
    [[nodiscard]] bool write_can(data::DataId field_id, const data::CanDataView& view) noexcept {
        return process_result(serializer_.write_can(field_id, view));
    }

    [[nodiscard]] bool write_uart(data::DataId field_id, const data::UartDataView& view) noexcept {
        return process_result(serializer_.write_uart(field_id, view));
    }

    [[nodiscard]] bool write_gpio_digital_data(
        uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
        if (view.timestamp_quarter_us.has_value()) [[unlikely]]
            return false;
        return process_result(serializer_.write_gpio_digital_value(channel_index, view));
    }

    [[nodiscard]] bool write_gpio_digital_read_config(
        uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
        return process_result(serializer_.write_gpio_digital_read_config(channel_index, view));
    }

    [[nodiscard]] bool write_gpio_analog_data(
        uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
        return process_result(serializer_.write_gpio_analog_value(channel_index, view));
    }

    [[nodiscard]] bool write_imu_accelerometer(const data::AccelerometerDataView& view) noexcept {
        return process_result(serializer_.write_imu_accelerometer(view));
    }

    [[nodiscard]] bool write_imu_gyroscope(const data::GyroscopeDataView& view) noexcept {
        return process_result(serializer_.write_imu_gyroscope(view));
    }

private:
    static bool process_result(core::protocol::Serializer::SerializeResult result) {
        using core::protocol::Serializer;
        if (result == Serializer::SerializeResult::kSuccess) [[likely]]
            return true;
        if (result == Serializer::SerializeResult::kBadAlloc) {
            logging::get_logger().error("Transmit buffer unavailable (acquire failed)");
            return true;
        }
        if (result == Serializer::SerializeResult::kInvalidArgument) {
            return false;
        }
        core::utility::assert_failed_debug();
    }

    StreamBuffer buffer_;
    core::protocol::Serializer serializer_;
};

} // namespace

Handler::PacketBuilder::PacketBuilder(void* transport_ptr) noexcept {
    static_assert(sizeof(PacketBuilderImpl) <= sizeof(storage_));
    static_assert(alignof(PacketBuilderImpl) <= alignof(std::uintptr_t));

    auto& transport_ref = *static_cast<transport::Transport*>(transport_ptr);
    std::construct_at(reinterpret_cast<PacketBuilderImpl*>(storage_), transport_ref);
}

Handler::PacketBuilder::~PacketBuilder() noexcept {
    std::destroy_at(std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_)));
}

bool Handler::PacketBuilder::write_can(
    data::DataId field_id, const data::CanDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_can(field_id, view);
}

bool Handler::PacketBuilder::write_uart(
    data::DataId field_id, const data::UartDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_uart(field_id, view);
}

bool Handler::PacketBuilder::write_gpio_digital_data(
    uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_digital_data(channel_index, view);
}

bool Handler::PacketBuilder::write_gpio_digital_read_config(
    uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_digital_read_config(channel_index, view);
}

bool Handler::PacketBuilder::write_gpio_analog_data(
    uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_analog_data(channel_index, view);
}

Handler::Handler(
    uint16_t usb_vid, int32_t usb_pid, std::string_view serial_filter,
    const board::AdvancedOptions& options, data::DataCallback& callback)
    : impl_(new Impl(
          transport::usb::create_transport(usb_vid, usb_pid, serial_filter, options), callback)) {}

Handler::Handler(Handler&& other) noexcept
    : impl_(std::exchange(other.impl_, nullptr)) {}

Handler& Handler::operator=(Handler&& other) noexcept {
    if (this == &other)
        return *this;
    delete impl_;
    impl_ = std::exchange(other.impl_, nullptr);
    return *this;
}

Handler::~Handler() noexcept { delete impl_; }

Handler::PacketBuilder Handler::start_transmit() noexcept {
    core::utility::assert_debug(impl_);
    return impl_->start_transmit();
}

} // namespace librmcs::host::protocol
