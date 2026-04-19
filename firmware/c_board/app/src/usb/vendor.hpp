#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

#include <class/vendor/vendor_device.h>
#include <device/usbd.h>
#include <tusb.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/include/librmcs/spec/c_board/gpio.hpp"
#include "core/include/librmcs/spec/gpio.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/gpio/gpio.hpp"
#include "firmware/c_board/app/src/i2c/i2c.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/c_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    static constexpr size_t kMaxPacketSize = 64;
    static constexpr std::size_t kGpioChannelCount = std::size(spec::c_board::kGpioDescriptors);

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
            transmit_buffer_.try_unlock();
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
        case data::DataId::kUart1: uart::uart1->handle_downlink(data); break;
        case data::DataId::kUart2: uart::uart2->handle_downlink(data); break;
        default: core::utility::assert_failed_always();
        }
    }

    void gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (channel_index >= kGpioChannelCount)
            return;

        const auto& gpio = spec::c_board::kGpioDescriptors[channel_index];
        if (!gpio.supports(spec::GpioCapability::kDigitalWrite))
            return;

        gpio::gpio->handle_digital_write(channel_index, data);
    }

    void gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (channel_index >= kGpioChannelCount)
            return;

        const auto& gpio = spec::c_board::kGpioDescriptors[channel_index];
        if (!gpio.supports(spec::GpioCapability::kAnalogWrite))
            return;

        gpio::gpio->handle_analog_write(channel_index, data);
    }

    void gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (channel_index >= kGpioChannelCount)
            return;

        const auto& gpio = spec::c_board::kGpioDescriptors[channel_index];
        if (!data.supported(gpio))
            return;

        gpio::gpio->handle_digital_read(channel_index, data);
    }

    void gpio_analog_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        (void)channel_index;
        (void)data;
    }

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override {
        (void)data;
    }

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override {
        (void)data;
    }

    void i2c_write_deserialized_callback(
        core::protocol::FieldId id, const data::I2cDataView& data) override {
        switch (id) {
        case data::DataId::kI2c0: i2c::i2c0->handle_downlink_write(data); break;
        default: core::utility::assert_failed_always();
        }
    }

    void i2c_read_config_deserialized_callback(
        core::protocol::FieldId id, const data::I2cReadConfigView& data) override {
        switch (id) {
        case data::DataId::kI2c0: i2c::i2c0->handle_downlink_read_config(data); break;
        default: core::utility::assert_failed_always();
        }
    }

    void i2c_read_result_deserialized_callback(
        core::protocol::FieldId id, const data::I2cDataView& data) override {
        // Downlink I2C read-result frames are ignored by design (uplink-only payload type).
        (void)id;
        (void)data;
    }

    void i2c_error_deserialized_callback(
        core::protocol::FieldId id, const data::I2cErrorView& data) override {
        // Downlink I2C error frames are ignored by design (uplink-only payload type).
        (void)id;
        (void)data;
    }

    void error_callback() override {
        // Drop malformed downlink payloads instead of trapping the whole device.
    }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
