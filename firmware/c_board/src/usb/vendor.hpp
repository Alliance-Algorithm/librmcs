#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>
#include <tusb.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/src/can/can.hpp"
#include "firmware/c_board/src/uart/uart.hpp"
#include "firmware/c_board/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/c_board/src/usb/usb_descriptors.hpp"
#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;

    Vendor() {
        usb::usb_descriptors.init();
        core::utility::assert_always(tusb_rhport_init(0, nullptr));
    }

    core::protocol::Serializer& serializer() { return serializer_; }

    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        deserializer_.feed(buffer);
        if (finished)
            deserializer_.finish_transfer();
    }

    bool try_transmit() {
        if (!tud_ready()) {
            transmit_buffer_.try_lock();
            return false;
        }

        if (!tud_vendor_n_write_available(0))
            return false;

        if (!transmitting_batch_) {
            transmit_buffer_.try_unlock_and_clear();
            transmitting_batch_ = transmit_buffer_.pop_batch();
        }
        if (!transmitting_batch_)
            return false;

        const auto data = transmitting_batch_->data();

        const auto target_size = std::min(data.size() - transmitted_size_, kMaxPacketSize);

        if (target_size) {
            const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            core::utility::assert_debug(tud_vendor_n_write_zlp(0));
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < kMaxPacketSize) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }

        return true;
    }

private:
    void can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        switch (id) {
        case data::DataId::kCan1: can::can1->handle_downlink(data); break;
        case data::DataId::kCan2: can::can2->handle_downlink(data); break;
        default: core::utility::assert_failed_always();
        }
    }

    void uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        switch (id) {
        case data::DataId::kUartDbus: uart::uart_dbus->handle_downlink(data); break;
        case data::DataId::kUart1: uart::uart1->handle_downlink(data); break;
        case data::DataId::kUart2: uart::uart2->handle_downlink(data); break;
        default: core::utility::assert_failed_always();
        }
    }

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override {
        (void)data;
    }

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override {
        (void)data;
    }

    void error_callback() override { core::utility::assert_failed_always(); }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
