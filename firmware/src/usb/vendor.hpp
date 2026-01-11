#pragma once

#include <algorithm>
#include <atomic>
#include <common/tusb_types.h>
#include <cstddef>
#include <cstdint>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/src/can/can.hpp"
#include "firmware/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class Vendor
    : private core::protocol::IDeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    Vendor() = default;

    core::protocol::Serializer& serializer() { return serializer_; }

    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        deserializer_.feed(buffer);
        if (finished)
            deserializer_.finish_transfer();
    }

    void can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        switch (id) {
        case data::DataId::CAN0: can::can0->handle_downlink(data); break;
        case data::DataId::CAN1: can::can1->handle_downlink(data); break;
        case data::DataId::CAN2: can::can2->handle_downlink(data); break;
        case data::DataId::CAN3: can::can3->handle_downlink(data); break;
        default: core::utility::assert_failed_always();
        }
    };

    void uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        (void)id;
        (void)data;
    };

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override {
        (void)data;
    };

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override {
        (void)data;
    };

    void error_callback() override { core::utility::assert_failed_always(); };

    bool try_transmit() {
        if (!device_ready())
            return false;

        if (!transmitting_batch_)
            transmitting_batch_ = transmit_buffer_.pop_batch();
        if (!transmitting_batch_)
            return false;

        const auto written_size =
            transmitting_batch_->written_size.load(std::memory_order::relaxed);

        const std::size_t max_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
        const auto target_size = std::min(written_size - transmitted_size_, max_packet_size);

        if (target_size) {
            auto data = reinterpret_cast<uint8_t*>(transmitting_batch_->data + transmitted_size_);
            auto sent = tud_vendor_n_write(0, data, target_size);
            core::utility::assert_debug(sent == target_size);
        } else {
            core::utility::assert_debug(tud_vendor_n_write_zlp(0));
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == written_size && target_size < max_packet_size) {
            transmitting_batch_->written_size.store(0, std::memory_order::relaxed);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }

        return true;
    }

private:
    static bool device_ready() { return tud_ready() && tud_vendor_n_write_available(0); }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_{};
    core::protocol::Serializer serializer_{transmit_buffer_};

    InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
