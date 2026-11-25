#pragma once

#include <cstdint>
#include <stdexcept>
#include <string_view>

#include <librmcs/board/common.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>
#include <librmcs/spec/rmcs_board_ultra/can.hpp>
#include <librmcs/spec/rmcs_board_ultra/uart.hpp>

namespace librmcs::board {

/**
 * @brief Host interface for RMCS Board Ultra.
 *
 * The callback must outlive the board. Receive callbacks can run before the board constructor
 * returns. Copy the payload if it needs to outlive the callback.
 */
class RmcsBoardUltra final {
public:
    class Callback : public data::DataCallback {
    public:
        struct Spec {
            using Can = spec::rmcs_board_ultra::CanDescriptor;
            static constexpr spec::rmcs_board_ultra::internal::CanDescriptors kCans{};
            using Uart = spec::rmcs_board_ultra::UartDescriptor;
            static constexpr spec::rmcs_board_ultra::internal::UartDescriptors kUarts{};
        };

        using Can = Spec::Can;

        struct View {
            using Can = data::CanDataView;
            using Uart = data::UartDataView;
            using ImuAccelerometer = data::ImuAccelerometerDataView;
            using ImuGyroscope = data::ImuGyroscopeDataView;
            using ImuTemperature = data::ImuTemperatureDataView;
        };

        virtual void can_receive_callback(const Spec::Can& can, const View::Can& data) {
            (void)can;
            (void)data;
        }

        bool can_receive_callback(data::DataId id, const data::CanDataView& data) final {
            const auto* descriptor = spec::rmcs_board_ultra::kCanDescriptors.find(id);
            if (descriptor == nullptr) [[unlikely]]
                return false;
            can_receive_callback(*descriptor, data);
            return true;
        }

        virtual void uart_receive_callback(const Spec::Uart&, const View::Uart&) {}

        bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
            const auto* descriptor = Spec::kUarts.find(id);
            if (descriptor == nullptr) [[unlikely]]
                return false;
            uart_receive_callback(*descriptor, data);
            return true;
        }
        bool gpio_digital_read_result_callback(uint8_t, const data::GpioDigitalDataView&) final {
            return false;
        }
        bool gpio_analog_read_result_callback(uint8_t, const data::GpioAnalogDataView&) final {
            return false;
        }
        void accelerometer_receive_callback(const View::ImuAccelerometer&) override {}
        void gyroscope_receive_callback(const View::ImuGyroscope&) override {}
        void temperature_receive_callback(const View::ImuTemperature&) override {}
    };

    explicit RmcsBoardUltra(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {})
        : handler_(0xA11C, 0xAF02, serial_filter, options, callback) {}

    explicit RmcsBoardUltra(
        const EthercatOptions& ethercat, Callback& callback = default_callback_,
        const AdvancedOptions& options = {})
        : handler_(ethercat, options, callback) {}

    RmcsBoardUltra(const RmcsBoardUltra&) = delete;
    RmcsBoardUltra& operator=(const RmcsBoardUltra&) = delete;
    RmcsBoardUltra(RmcsBoardUltra&&) = delete;
    RmcsBoardUltra& operator=(RmcsBoardUltra&&) = delete;
    ~RmcsBoardUltra() = default;

    // An EtherCAT session does not reconnect in place. Reconstruct after link loss.
    bool connected() const noexcept { return handler_.connected(); }

    class PacketBuilder {
        friend class RmcsBoardUltra;

    public:
        PacketBuilder& can_transmit(
            const spec::rmcs_board_ultra::CanDescriptor& can, const data::CanDataView& data) {
            if (!builder_.write_can(can.data_id, data)) [[unlikely]]
                throw std::invalid_argument{"CAN transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder&
            uart_transmit(const Callback::Spec::Uart& port, const data::UartDataView& data) {
            if (port == Callback::Spec::kUarts.kDbus) [[unlikely]]
                throw std::invalid_argument{"DBUS is receive-only"};
            if (!builder_.write_uart(port.data_id, data)) [[unlikely]]
                throw std::invalid_argument{"UART transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder&
            uart_config(const Callback::Spec::Uart& port, const data::UartConfigView& data) {
            if (!builder_.write_uart_config(port.config_data_id, data)) [[unlikely]]
                throw std::invalid_argument{"UART configuration failed: Invalid UART config"};
            return *this;
        }

    private:
        explicit PacketBuilder(host::protocol::Handler& handler) noexcept
            : builder_(handler.start_transmit()) {}

        host::protocol::Handler::PacketBuilder builder_;
    };

    PacketBuilder start_transmit() noexcept { return PacketBuilder{handler_}; }

    bool try_can_transmit(const Callback::Can& can, const data::CanDataView& frame) noexcept {
        return connected() && handler_.start_transmit().try_write_can(can.data_id, frame);
    }

private:
    static inline Callback default_callback_{};
    host::protocol::Handler handler_;
};

} // namespace librmcs::board
