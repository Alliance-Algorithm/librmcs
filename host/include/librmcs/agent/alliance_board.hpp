#pragma once

#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>

namespace librmcs::agent {

class AllianceBoard : private data::IDataCallback {
public:
    AllianceBoard()
        : handler_(0xa11c, -1, nullptr, *this) {}

    class PacketBuilder {
        friend class AllianceBoard;

    public:
        PacketBuilder& can0_transmit(const librmcs::data::CanDataView& data) {
            builder_.write_can(data::DataId::CAN0, data);
            return *this;
        }
        PacketBuilder& can1_transmit(const librmcs::data::CanDataView& data) {
            builder_.write_can(data::DataId::CAN1, data);
            return *this;
        }
        PacketBuilder& can2_transmit(const librmcs::data::CanDataView& data) {
            builder_.write_can(data::DataId::CAN2, data);
            return *this;
        }
        PacketBuilder& can3_transmit(const librmcs::data::CanDataView& data) {
            builder_.write_can(data::DataId::CAN3, data);
            return *this;
        }

        PacketBuilder& dbus_transmit(const librmcs::data::UartDataView& data) {
            builder_.write_uart(data::DataId::UART_DBUS, data);
            return *this;
        }
        PacketBuilder& uart0_transmit(const librmcs::data::UartDataView& data) {
            builder_.write_uart(data::DataId::UART0, data);
            return *this;
        }
        PacketBuilder& uart1_transmit(const librmcs::data::UartDataView& data) {
            builder_.write_uart(data::DataId::UART1, data);
            return *this;
        }
        PacketBuilder& uart2_transmit(const librmcs::data::UartDataView& data) {
            builder_.write_uart(data::DataId::UART2, data);
            return *this;
        }
        PacketBuilder& uart3_transmit(const librmcs::data::UartDataView& data) {
            builder_.write_uart(data::DataId::UART3, data);
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
        case data::DataId::CAN0: can0_receive_callback(data); return true;
        case data::DataId::CAN1: can1_receive_callback(data); return true;
        case data::DataId::CAN2: can2_receive_callback(data); return true;
        case data::DataId::CAN3: can3_receive_callback(data); return true;
        default: return false;
        }
    }

    virtual void can0_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can1_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can2_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }
    virtual void can3_receive_callback(const librmcs::data::CanDataView& data) { (void)data; }

    bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
        switch (id) {
        case data::DataId::UART_DBUS: dbus_receive_callback(data); return true;
        case data::DataId::UART0: uart0_receive_callback(data); return true;
        case data::DataId::UART1: uart1_receive_callback(data); return true;
        case data::DataId::UART2: uart2_receive_callback(data); return true;
        case data::DataId::UART3: uart3_receive_callback(data); return true;
        default: return false;
        }
    }

    virtual void dbus_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart0_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart1_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart2_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }
    virtual void uart3_receive_callback(const librmcs::data::UartDataView& data) { (void)data; }

    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        (void)data;
    }
    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        (void)data;
    }

    host::protocol::Handler handler_;
};

} // namespace librmcs::agent
