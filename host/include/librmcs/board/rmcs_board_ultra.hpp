#pragma once

#include <cstdint>
#include <string_view>

#include <librmcs/board/common.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>

namespace librmcs::board {

// The callback must outlive the board and may be invoked during construction.
class RmcsBoardUltra final {
public:
    class Callback : public data::DataCallback {
    public:
        struct View {
            using ImuAccelerometer = data::ImuAccelerometerDataView;
            using ImuGyroscope = data::ImuGyroscopeDataView;
            using ImuTemperature = data::ImuTemperatureDataView;
        };

        void accelerometer_receive_callback(const View::ImuAccelerometer&) override {}
        void gyroscope_receive_callback(const View::ImuGyroscope&) override {}
        void temperature_receive_callback(const View::ImuTemperature&) override {}

        bool can_receive_callback(data::DataId, const data::CanDataView&) final { return false; }
        bool uart_receive_callback(data::DataId, const data::UartDataView&) final { return false; }
        bool gpio_digital_read_result_callback(uint8_t, const data::GpioDigitalDataView&) final {
            return false;
        }
        bool gpio_analog_read_result_callback(uint8_t, const data::GpioAnalogDataView&) final {
            return false;
        }
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

private:
    static inline Callback default_callback_{};
    host::protocol::Handler handler_;
};

} // namespace librmcs::board
