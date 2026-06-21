#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <librmcs/spec/gpio.hpp>

namespace librmcs::data {

enum class DataId : uint8_t {
    kExtend = 0,

    kGpio = 1,

    kCan0 = 2,
    kCan1 = 3,
    kCan2 = 4,
    kCan3 = 5,
    kCan4 = 6,
    kCan5 = 7,
    kCan6 = 8,
    kCan7 = 9,

    kUartDbus = 10,
    kUart0 = 11,
    kUart1 = 12,
    kUart2 = 13,
    kUart3 = 14,

    kImu = 15,

    kSession = 16,
};

enum class SessionType : uint8_t {
    kStart = 0,
    kStartAck = 1,
    kKeepalive = 2,
    kKeepaliveAck = 3,
};

struct SessionControlView {
    SessionType type;
    uint32_t nonce;
};

struct CanDataView {
    uint32_t can_id;
    std::span<const std::byte> can_data;
    bool is_fdcan = false;
    bool is_extended_can_id = false;
    bool is_remote_transmission = false;
};

struct UartDataView {
    std::span<const std::byte> uart_data;
    bool idle_delimited = false;
};

struct GpioDigitalDataView {
    bool high;
    std::optional<uint32_t> timestamp_quarter_us = std::nullopt;
};

struct GpioAnalogDataView {
    uint16_t value;
};

enum class GpioPull : uint8_t {
    kNone = 0,
    kUp = 1,
    kDown = 2,
};

struct GpioReadConfigView {
    uint16_t period_ms = 0;
    bool asap = false;
    bool rising_edge = false;
    bool falling_edge = false;
    bool capture_timestamp = false;
    GpioPull pull = GpioPull::kNone;

    [[nodiscard]] constexpr bool supported(const spec::GpioDescriptor& gpio) const noexcept {
        return (!asap || gpio.supports(spec::GpioCapability::kDigitalReadOnce))
            && (!period_ms || gpio.supports(spec::GpioCapability::kDigitalReadPeriodic))
            && ((!rising_edge && !falling_edge)
                || gpio.supports(spec::GpioCapability::kDigitalReadInterrupt))
            && (pull != GpioPull::kUp || gpio.supports(spec::GpioCapability::kPullUp))
            && (pull != GpioPull::kDown || gpio.supports(spec::GpioCapability::kPullDown))
            && (!capture_timestamp || gpio.supports(spec::GpioCapability::kTimestampedDigitalRead));
    }
};

struct AccelerometerDataView {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp_quarter_us;
};

struct GyroscopeDataView {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp_quarter_us;
};

struct TemperatureDataView {
    uint16_t raw_register_value;
    uint32_t timestamp_quarter_us;
};

/**
 * @brief Interface for consuming deserialized uplink data.
 *
 * This interface is invoked after the protocol layer has already identified the payload type and
 * decoded its contents. For callback families that are further multiplexed by a sub-identifier,
 * such as `DataId` or a GPIO `channel_index`, the callback returns `bool` to report whether that
 * sub-identifier is valid for the concrete implementation.
 *
 * Return `true` when the sub-identifier is recognized and the payload has been dispatched.
 * Return `false` when deserialization succeeded but the `DataId` or `channel_index` is unexpected,
 * so the caller can propagate that routing error to upper layers.
 *
 * IMU callbacks return `void` because each payload type maps to a single callback and requires no
 * additional route validation.
 */
class DataCallback {
public:
    DataCallback() = default;
    DataCallback(const DataCallback&) = delete;
    DataCallback& operator=(const DataCallback&) = delete;
    DataCallback(DataCallback&&) = delete;
    DataCallback& operator=(DataCallback&&) = delete;
    virtual ~DataCallback() = default;

    [[nodiscard]] virtual bool can_receive_callback(DataId id, const CanDataView& data) = 0;

    [[nodiscard]] virtual bool uart_receive_callback(DataId id, const UartDataView& data) = 0;

    [[nodiscard]] virtual bool gpio_digital_read_result_callback(
        uint8_t channel_index, const GpioDigitalDataView& data) = 0;
    [[nodiscard]] virtual bool
        gpio_analog_read_result_callback(uint8_t channel_index, const GpioAnalogDataView& data) = 0;

    virtual void accelerometer_receive_callback(const AccelerometerDataView& data) = 0;
    virtual void gyroscope_receive_callback(const GyroscopeDataView& data) = 0;
    virtual void temperature_receive_callback(const TemperatureDataView& data) = 0;
};

} // namespace librmcs::data
