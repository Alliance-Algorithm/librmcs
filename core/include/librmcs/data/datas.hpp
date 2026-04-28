#pragma once

#include <cstddef>
#include <cstdint>
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
    GpioPull pull = GpioPull::kNone;

    [[nodiscard]] constexpr bool supported(const spec::GpioDescriptor& gpio) const noexcept {
        return (!asap || gpio.supports(spec::GpioCapability::kDigitalReadOnce))
            && (!period_ms || gpio.supports(spec::GpioCapability::kDigitalReadPeriodic))
            && ((!rising_edge && !falling_edge)
                || gpio.supports(spec::GpioCapability::kDigitalReadInterrupt))
            && (pull != GpioPull::kUp || gpio.supports(spec::GpioCapability::kPullUp))
            && (pull != GpioPull::kDown || gpio.supports(spec::GpioCapability::kPullDown));
    }
};

struct AccelerometerDataView {
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t timestamp_diff_quarter_us;
};

struct GyroscopeDataView {
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t timestamp_diff_quarter_us;
};

class DataCallback {
public:
    DataCallback() = default;
    DataCallback(const DataCallback&) = delete;
    DataCallback& operator=(const DataCallback&) = delete;
    DataCallback(DataCallback&&) = delete;
    DataCallback& operator=(DataCallback&&) = delete;
    virtual ~DataCallback() = default;

    // `*_receive_callback` returns `true` if id is valid
    virtual bool can_receive_callback(DataId id, const CanDataView& data) = 0;

    virtual bool uart_receive_callback(DataId id, const UartDataView& data) = 0;

    virtual void gpio_digital_read_result_callback(
        uint8_t channel_index, const GpioDigitalDataView& data) = 0;

    virtual void
        gpio_analog_read_result_callback(uint8_t channel_index, const GpioAnalogDataView& data) = 0;

    virtual void accelerometer_receive_callback(const AccelerometerDataView& data) = 0;

    virtual void gyroscope_receive_callback(const GyroscopeDataView& data) = 0;
};

} // namespace librmcs::data
