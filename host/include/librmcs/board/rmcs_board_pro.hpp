#pragma once

#include <cstdint>
#include <stdexcept>
#include <string_view>

#include <librmcs/board/common.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>
#include <librmcs/spec/gpio.hpp>
#include <librmcs/spec/rmcs_board_pro/can.hpp>
#include <librmcs/spec/rmcs_board_pro/gpio.hpp>
#include <librmcs/spec/rmcs_board_pro/uart.hpp>

namespace librmcs::board {

/**
 * @brief High-level host board interface for RMCS Board Pro.
 *
 * This class owns the transport and protocol stack for a single board connection.
 * The supplied `Callback` is stored by reference, is not owned by the board, and must outlive the
 * board instance.
 *
 * The board may start transport I/O during construction, so receive callbacks may be invoked before
 * the board constructor returns.
 *
 * A common usage pattern is for an enclosing user type to inherit `Callback` and declare the board
 * as its last data member. In that arrangement, early callbacks may access base subobjects and
 * members whose initialization has already completed before the board member begins construction.
 *
 * @warning Early callbacks must not depend on invariants established later in an enclosing
 * constructor body, on post-construction configuration, or on the board object itself having
 * finished construction. Delay board construction with `std::optional` or `std::unique_ptr` when
 * callback behavior depends on such state.
 */
class RmcsBoardPro final {
public:
    class Callback : public data::DataCallback {
    public:
        struct Spec {
            using Can = spec::rmcs_board_pro::CanDescriptor;
            static constexpr spec::rmcs_board_pro::internal::CanDescriptors kCans{};

            using Uart = spec::rmcs_board_pro::UartDescriptor;
            static constexpr spec::rmcs_board_pro::internal::UartDescriptors kUarts{};

            using Gpio = spec::rmcs_board_pro::GpioDescriptor;
            static constexpr spec::rmcs_board_pro::internal::GpioDescriptors kGpios{};
        };

        struct View {
            using Can = data::CanDataView;

            using Uart = data::UartDataView;

            using GpioDigital = data::GpioDigitalDataView;

            using ImuAccelerometer = librmcs::data::ImuAccelerometerDataView;
            using ImuGyroscope = librmcs::data::ImuGyroscopeDataView;
            using ImuTemperature = librmcs::data::ImuTemperatureDataView;
        };

        virtual void can_receive_callback(const Spec::Can& can, const View::Can& data) {
            (void)can;
            (void)data;
        }

        virtual void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) {
            (void)uart;
            (void)data;
        }

        virtual void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) {
            (void)gpio;
            (void)data;
        }
        virtual void gpio_analog_read_result_callback(
            const Spec::Gpio& gpio, const librmcs::data::GpioAnalogDataView& data) {
            (void)gpio;
            (void)data;
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            (void)data;
        }
        void gyroscope_receive_callback(const View::ImuGyroscope& data) override { (void)data; }
        void temperature_receive_callback(const View::ImuTemperature& data) override { (void)data; }

    public:
        bool can_receive_callback(data::DataId id, const data::CanDataView& data) final {
            const auto* descriptor = spec::rmcs_board_pro::kCanDescriptors.find(id);
            if (descriptor == nullptr) [[unlikely]]
                return false;
            can_receive_callback(*descriptor, data);
            return true;
        }

        bool uart_receive_callback(data::DataId id, const data::UartDataView& data) final {
            const auto* descriptor = spec::rmcs_board_pro::kUartDescriptors.find(id);
            if (descriptor == nullptr) [[unlikely]]
                return false;
            uart_receive_callback(*descriptor, data);
            return true;
        }

        bool gpio_digital_read_result_callback(
            uint8_t channel_index, const data::GpioDigitalDataView& data) final {
            if (channel_index >= spec::rmcs_board_pro::kGpioDescriptors.size()) [[unlikely]]
                return false;
            gpio_digital_read_result_callback(
                spec::rmcs_board_pro::kGpioDescriptors[channel_index], data);
            return true;
        }

        bool gpio_analog_read_result_callback(
            uint8_t channel_index, const data::GpioAnalogDataView& data) final {
            if (channel_index >= spec::rmcs_board_pro::kGpioDescriptors.size()) [[unlikely]]
                return false;
            gpio_analog_read_result_callback(
                spec::rmcs_board_pro::kGpioDescriptors[channel_index], data);
            return true;
        }
    };

    explicit RmcsBoardPro(
        Callback& callback = default_callback_, std::string_view serial_filter = {},
        const AdvancedOptions& options = {})
        : handler_(0xA11C, 0xAF01, serial_filter, options, callback) {}

    RmcsBoardPro(const RmcsBoardPro&) = delete;
    RmcsBoardPro& operator=(const RmcsBoardPro&) = delete;
    RmcsBoardPro(RmcsBoardPro&&) = delete;
    RmcsBoardPro& operator=(RmcsBoardPro&&) = delete;
    ~RmcsBoardPro() = default;

    class PacketBuilder {
        friend class RmcsBoardPro;

    public:
        PacketBuilder& can_transmit(
            const librmcs::spec::rmcs_board_pro::CanDescriptor& can,
            const librmcs::data::CanDataView& data) {
            if (!builder_.write_can(can.data_id, data)) [[unlikely]]
                throw std::invalid_argument{"CAN transmission failed: Invalid CAN data"};
            return *this;
        }

        PacketBuilder& uart_transmit(
            const librmcs::spec::rmcs_board_pro::UartDescriptor& uart,
            const librmcs::data::UartDataView& data) {
            if (!builder_.write_uart(uart.data_id, data)) [[unlikely]]
                throw std::invalid_argument{"UART transmission failed: Invalid UART data"};
            return *this;
        }

        PacketBuilder& uart_config(
            const librmcs::spec::rmcs_board_pro::UartDescriptor& uart,
            const librmcs::data::UartConfigView& data) {
            if (!builder_.write_uart_config(uart.config_data_id, data)) [[unlikely]]
                throw std::invalid_argument{
                    "UART configuration transmission failed: Invalid UART config"};
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
    static inline Callback default_callback_{};
    host::protocol::Handler handler_;
};

} // namespace librmcs::board
