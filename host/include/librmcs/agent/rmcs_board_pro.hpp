#pragma once

#include <cstdint>
#include <stdexcept>
#include <string_view>

#include <librmcs/agent/common.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>
#include <librmcs/spec/gpio.hpp>
#include <librmcs/spec/rmcs_board_pro/gpio.hpp>

namespace librmcs::agent {

class RmcsBoardPro : private data::DataCallback {
public:
    explicit RmcsBoardPro(std::string_view serial_filter = {}, const AdvancedOptions& options = {})
        : handler_(0xA11C, 0xAF01, serial_filter, options, *this) {}

    RmcsBoardPro(const RmcsBoardPro&) = delete;
    RmcsBoardPro& operator=(const RmcsBoardPro&) = delete;
    RmcsBoardPro(RmcsBoardPro&&) = delete;
    RmcsBoardPro& operator=(RmcsBoardPro&&) = delete;
    ~RmcsBoardPro() override = default;

    class PacketBuilder {
        friend class RmcsBoardPro;

    public:
        PacketBuilder& can0_transmit(const librmcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan0, data)) [[unlikely]]
                throw std::invalid_argument{"CAN0 transmission failed: Invalid CAN data"};
            return *this;
        }
        PacketBuilder& can1_transmit(const librmcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan1, data)) [[unlikely]]
                throw std::invalid_argument{"CAN1 transmission failed: Invalid CAN data"};
            return *this;
        }
        PacketBuilder& can2_transmit(const librmcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan2, data)) [[unlikely]]
                throw std::invalid_argument{"CAN2 transmission failed: Invalid CAN data"};
            return *this;
        }
        PacketBuilder& can3_transmit(const librmcs::data::CanDataView& data) {
            if (!builder_.write_can(data::DataId::kCan3, data)) [[unlikely]]
                throw std::invalid_argument{"CAN3 transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder& uart0_transmit(const librmcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart0, data)) [[unlikely]]
                throw std::invalid_argument{"UART0 transmission failed: Invalid UART data"};
            return *this;
        }
        PacketBuilder& uart1_transmit(const librmcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart1, data)) [[unlikely]]
                throw std::invalid_argument{"UART1 transmission failed: Invalid UART data"};
            return *this;
        }
        PacketBuilder& uart2_transmit(const librmcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart2, data)) [[unlikely]]
                throw std::invalid_argument{"UART2 transmission failed: Invalid UART data"};
            return *this;
        }
        PacketBuilder& uart3_transmit(const librmcs::data::UartDataView& data) {
            if (!builder_.write_uart(data::DataId::kUart3, data)) [[unlikely]]
                throw std::invalid_argument{"UART3 transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder& gpio_digital_write(
            const librmcs::spec::rmcs_board_pro::GpioDescriptor& gpio,
            const librmcs::data::GpioDigitalDataView& data) {
            if (!gpio.supports(spec::GpioCapability::kDigitalWrite)
                || !builder_.write_gpio_digital_data(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{"GPIO digital transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_digital_read(
            const librmcs::spec::rmcs_board_pro::GpioDescriptor& gpio,
            const librmcs::data::GpioReadConfigView& data) {
            if (!data.supported(gpio)
                || !builder_.write_gpio_digital_read_config(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{
                    "GPIO digital read configuration transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_analog_write(
            const librmcs::spec::rmcs_board_pro::GpioDescriptor& gpio,
            const librmcs::data::GpioAnalogDataView& data) {
            if (!gpio.supports(spec::GpioCapability::kAnalogWrite)
                || !builder_.write_gpio_analog_data(gpio.channel_index, data)) [[unlikely]]
                throw std::invalid_argument{"GPIO analog transmission failed: Invalid GPIO data"};
            return *this;
        }

    private:
        explicit PacketBuilder(host::protocol::Handler& handler) noexcept
            : builder_(handler.start_transmit()) {}

        host::protocol::Handler::PacketBuilder builder_;
    };
    PacketBuilder start_transmit() noexcept { return PacketBuilder{handler_}; }

private:
    bool can_receive_callback(data::DataId id, const data::CanDataView& data) final {
        switch (id) {
        case data::DataId::kCan0: can0_receive_callback(data); return true;
        case data::DataId::kCan1: can1_receive_callback(data); return true;
        case data::DataId::kCan2: can2_receive_callback(data); return true;
        case data::DataId::kCan3: can3_receive_callback(data); return true;
        default: return false;
        }
    }

    virtual void can0_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can1_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can2_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can3_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }

    bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
        switch (id) {
        case data::DataId::kUartDbus: dbus_receive_callback(data); return true;
        case data::DataId::kUart0: uart0_receive_callback(data); return true;
        case data::DataId::kUart1: uart1_receive_callback(data); return true;
        case data::DataId::kUart2: uart2_receive_callback(data); return true;
        case data::DataId::kUart3: uart3_receive_callback(data); return true;
        default: return false;
        }
    }

    virtual void dbus_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart0_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart1_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart2_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart3_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }

    void gpio_digital_read_result_callback(
        uint8_t channel_index, const librmcs::data::GpioDigitalDataView& data) final {
        if (channel_index >= spec::rmcs_board_pro::kGpioDescriptors.size()) [[unlikely]]
            return;
        gpio_digital_read_result_callback(
            spec::rmcs_board_pro::kGpioDescriptors[channel_index], data);
    }
    void gpio_analog_read_result_callback(
        uint8_t channel_index, const librmcs::data::GpioAnalogDataView& data) final {
        if (channel_index >= spec::rmcs_board_pro::kGpioDescriptors.size()) [[unlikely]]
            return;
        gpio_analog_read_result_callback(
            spec::rmcs_board_pro::kGpioDescriptors[channel_index], data);
    }

    virtual void gpio_digital_read_result_callback(
        const librmcs::spec::rmcs_board_pro::GpioDescriptor& gpio,
        const librmcs::data::GpioDigitalDataView& data) {
        (void)gpio;
        (void)data;
    }
    virtual void gpio_analog_read_result_callback(
        const librmcs::spec::rmcs_board_pro::GpioDescriptor& gpio,
        const librmcs::data::GpioAnalogDataView& data) {
        (void)gpio;
        (void)data;
    }

    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        (void)data;
    }
    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        (void)data;
    }
    void temperature_receive_callback(const librmcs::data::TemperatureDataView& data) override {
        (void)data;
    }

    host::protocol::Handler handler_;
};

} // namespace librmcs::agent
