#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <librmcs/data/datas.hpp>

#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/peripherals.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"

namespace librmcs::firmware {

class Session final : private core::protocol::DeserializeCallback {
public:
    using ResetTransmit = void (*)(void*);

    Session(core::protocol::Serializer& serializer, ResetTransmit reset_transmit, void* context)
        : serializer_(serializer)
        , reset_transmit_(reset_transmit)
        , context_(context) {}

    void feed(std::span<const std::byte> bytes, bool finished) {
        deserializer_.feed(bytes);
        if (finished)
            finish_transfer();
    }

    void finish_transfer() { deserializer_.finish_transfer(); }
    void deactivate() { established_ = false; }
    bool established() const { return established_; }

    bool expire() {
        constexpr uint64_t lease_quarter_us = 4'000'000;
        if (!established_ || timer::Timer::timestamp64_quarter_us() - refreshed_ < lease_quarter_us)
            return false;

        deactivate();
        return true;
    }

private:
    bool can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        return !established_ || Peripherals::write_can(id, data);
    }

    bool uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        return !established_ || Peripherals::write_uart(id, data);
    }

    bool uart_config_deserialized_callback(
        core::protocol::FieldId id, const data::UartConfigView& data) override {
        return !established_ || Peripherals::configure_uart(id, data);
    }

    bool gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        return !established_ || Peripherals::write_gpio_digital(channel_index, data);
    }

    bool gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        return !established_ || Peripherals::write_gpio_analog(channel_index, data);
    }

    bool gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        return !established_ || Peripherals::configure_gpio_digital(channel_index, data);
    }

    bool gpio_analog_read_config_deserialized_callback(
        uint8_t, const data::GpioReadConfigView&) override {
        return !established_;
    }

    void accelerometer_deserialized_callback(const data::ImuAccelerometerDataView&) override {}
    void gyroscope_deserialized_callback(const data::ImuGyroscopeDataView&) override {}
    void temperature_deserialized_callback(const data::ImuTemperatureDataView&) override {}

    void session_control_deserialized_callback(const data::SessionControlView& control) override {
        data::SessionType acknowledgement;
        switch (control.type) {
        case data::SessionType::kStart:
            if (!established_ || control.nonce != nonce_) {
                reset_transmit_(context_);
                nonce_ = control.nonce;
                established_ = true;
            }
            acknowledgement = data::SessionType::kStartAck;
            break;
        case data::SessionType::kKeepalive:
            if (!established_ || control.nonce != nonce_)
                return;
            acknowledgement = data::SessionType::kKeepaliveAck;
            break;
        default: return;
        }

        refreshed_ = timer::Timer::timestamp64_quarter_us();
        const auto result = serializer_.write_session_control({acknowledgement, control.nonce});
        core::utility::assert_always(
            result != core::protocol::Serializer::SerializeResult::kInvalidArgument);
    }

    void error_callback() override {}

    core::protocol::Deserializer deserializer_{*this};
    core::protocol::Serializer& serializer_;
    ResetTransmit reset_transmit_;
    void* context_;
    uint64_t refreshed_ = 0;
    uint32_t nonce_ = 0;
    bool established_ = false;
};

} // namespace librmcs::firmware
