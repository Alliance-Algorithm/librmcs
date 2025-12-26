#pragma once

#include <atomic>
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

        auto batch = transmit_buffer_.pop_batch();
        if (!batch)
            return false;

        auto written_size = batch->written_size.load(std::memory_order::relaxed);
        batch->written_size.store(0, std::memory_order::relaxed);

        auto data = reinterpret_cast<uint8_t*>(batch->data);
        auto sent = tud_vendor_n_write(0, data, written_size);

        return sent == written_size;
    }

private:
    static bool device_ready() { return tud_ready() && tud_vendor_n_write_available(0); }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_{};
    core::protocol::Serializer serializer_{transmit_buffer_};
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
