#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include <class/vendor/vendor_device.h>
#include <common/tusb_types.h>
#include <device/usbd.h>
#include <tusb.h>

#include "board_app.hpp"
#include "core/include/librmcs/data/datas.hpp"
#include "core/include/librmcs/spec/gpio.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/can/can.hpp"
#include "firmware/rmcs_board/app/src/gpio/gpio.hpp"
#include "firmware/rmcs_board/app/src/uart/uart.hpp"
#include "firmware/rmcs_board/app/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/rmcs_board/app/src/usb/usb_descriptors.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    Vendor() {
        usb::usb_descriptors.init();

        board::init_user_button_and_switch_pins();
        const tusb_rhport_init_t init_config{
            .role = TUSB_ROLE_DEVICE,
            .speed = board::kUserHsFsSwitchPin.is_active() ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL,
        };
        core::utility::assert_always(tusb_rhport_init(0, &init_config));
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

        const std::size_t max_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
        const auto target_size = std::min(data.size() - transmitted_size_, max_packet_size);

        if (target_size) {
            const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            core::utility::assert_debug(tud_vendor_n_write_zlp(0));
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < max_packet_size) {
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
        case data::DataId::kCan0: can::can_array[0]->handle_downlink(data); break;
        case data::DataId::kCan1: can::can_array[1]->handle_downlink(data); break;
        case data::DataId::kCan2: can::can_array[2]->handle_downlink(data); break;
        case data::DataId::kCan3: can::can_array[3]->handle_downlink(data); break;
        default: core::utility::assert_failed_always();
        }
    }

    void uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        switch (id) {
        case data::DataId::kUart0: uart::uart_array[0]->handle_downlink(data); break;
        case data::DataId::kUart1: uart::uart_array[1]->handle_downlink(data); break;
#ifdef BOARD_UART2
        case data::DataId::kUart2: uart::uart_array[2]->handle_downlink(data); break;
#endif
#ifdef BOARD_UART3
        case data::DataId::kUart3: uart::uart_array[3]->handle_downlink(data); break;
#endif
        default: core::utility::assert_failed_always();
        }
    }

    void gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        if (channel_index >= board::spec::kGpioDescriptors.size())
            return;

        const auto& gpio_descriptor = board::spec::kGpioDescriptors[channel_index];
        if (!gpio_descriptor.supports(spec::GpioCapability::kDigitalWrite))
            return;

        gpio::gpio->handle_digital_write(channel_index, data);
    }

    void gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        if (channel_index >= board::spec::kGpioDescriptors.size())
            return;

        const auto& gpio_descriptor = board::spec::kGpioDescriptors[channel_index];
        if (!gpio_descriptor.supports(spec::GpioCapability::kAnalogWrite))
            return;

        gpio::gpio->handle_analog_write(channel_index, data);
    }

    void gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        if (channel_index >= board::spec::kGpioDescriptors.size())
            return;

        const auto& gpio_descriptor = board::spec::kGpioDescriptors[channel_index];
        if (!data.supported(gpio_descriptor))
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

    void error_callback() override { core::utility::assert_failed_always(); }

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
