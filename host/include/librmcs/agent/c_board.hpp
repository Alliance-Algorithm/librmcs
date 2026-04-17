#pragma once

#include <cstdint>
#include <stdexcept>
#include <string_view>

#include <librmcs/agent/common.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>
#include <librmcs/protocol/i2c.hpp>

namespace librmcs::agent {

class CBoard : private data::DataCallback {
public:
    explicit CBoard(std::string_view serial_filter = {}, const AdvancedOptions& options = {})
        : handler_(0xA11C, 0xD401, serial_filter, options, *this) {}

    CBoard(const CBoard&) = delete;
    CBoard& operator=(const CBoard&) = delete;
    CBoard(CBoard&&) = delete;
    CBoard& operator=(CBoard&&) = delete;
    ~CBoard() override = default;

    class PacketBuilder {
        friend class CBoard;

    public:
        static constexpr uint16_t kI2cMaxDataLength = librmcs::protocol::kI2cMaxDataLength;

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

        PacketBuilder& i2c0_write(const librmcs::data::I2cDataView& data) {
            if (data.payload.empty() || data.payload.size() > kI2cMaxDataLength
                || data.slave_address > 0x7FU) [[unlikely]]
                throw std::invalid_argument{"I2C0 write failed: Invalid I2C data"};
            if (!builder_.write_i2c(data::DataId::kI2c0, data)) [[unlikely]]
                throw std::runtime_error{"I2C0 write failed: Transmit buffer unavailable"};
            return *this;
        }

        PacketBuilder& i2c0_read(const librmcs::data::I2cReadConfigView& data) {
            if (data.read_length == 0 || data.read_length > kI2cMaxDataLength
                || data.slave_address > 0x7FU) [[unlikely]]
                throw std::invalid_argument{"I2C0 read failed: Invalid I2C read config"};
            if (!builder_.write_i2c_read_config(data::DataId::kI2c0, data)) [[unlikely]]
                throw std::runtime_error{"I2C0 read failed: Transmit buffer unavailable"};
            return *this;
        }

        PacketBuilder& gpio_digital_write(const librmcs::data::GpioDigitalDataView& data) {
            if (data.channel < 1 || data.channel > 7 || !builder_.write_gpio_digital_data(data))
                [[unlikely]]
                throw std::invalid_argument{"GPIO digital transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_digital_read(const librmcs::data::GpioReadConfigView& data) {
            if (data.channel < 1 || data.channel > 7
                || (data.channel == 6 && (data.rising_edge || data.falling_edge))
                || !builder_.write_gpio_digital_read_config(data)) [[unlikely]]
                throw std::invalid_argument{
                    "GPIO digital read configuration transmission failed: Invalid GPIO data"};
            return *this;
        }
        PacketBuilder& gpio_analog_write(const librmcs::data::GpioAnalogDataView& data) {
            if (data.channel < 1 || data.channel > 7 || !builder_.write_gpio_analog_data(data))
                [[unlikely]]
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
        case data::DataId::kCan1: can1_receive_callback(data); return true;
        case data::DataId::kCan2: can2_receive_callback(data); return true;
        default: return false;
        }
    }

    virtual void can1_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can2_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }

    bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
        switch (id) {
        case data::DataId::kUartDbus: dbus_receive_callback(data); return true;
        case data::DataId::kUart1: uart1_receive_callback(data); return true;
        case data::DataId::kUart2: uart2_receive_callback(data); return true;
        default: return false;
        }
    }

    virtual void dbus_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart1_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart2_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }

    void
        gpio_digital_read_result_callback(const librmcs::data::GpioDigitalDataView& data) override {
        (void)data;
    }
    void gpio_analog_read_result_callback(const librmcs::data::GpioAnalogDataView& data) override {
        (void)data;
    }

    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        (void)data;
    }
    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        (void)data;
    }

    bool i2c_receive_callback(data::DataId id, const data::I2cDataView& data) final {
        switch (id) {
        case data::DataId::kI2c0: i2c0_receive_callback(data); return true;
        default: return false;
        }
    }

    void i2c_error_callback(data::DataId id, const data::I2cErrorView& data) final {
        switch (id) {
        case data::DataId::kI2c0: i2c0_error_callback(data); break;
        default: break;
        }
    }

protected:
    virtual void i2c0_receive_callback(const librmcs::data::I2cDataView& data) { (void)data; }

    virtual void i2c0_error_callback(const librmcs::data::I2cErrorView& data) { (void)data; }

    void i2c0_error_from_slave_address(uint8_t slave_address) {
        data::DataCallback::i2c_error_from_slave_address(data::DataId::kI2c0, slave_address);
    }

private:
    host::protocol::Handler handler_;
};

} // namespace librmcs::agent
