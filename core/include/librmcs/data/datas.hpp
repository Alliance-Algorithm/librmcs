#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace librmcs::data {

enum class DataId : uint8_t {
    EXTEND = 0,

    GPIO = 1,

    CAN0 = 2,
    CAN1 = 3,
    CAN2 = 4,
    CAN3 = 5,
    CAN4 = 6,
    CAN5 = 7,
    CAN6 = 8,
    CAN7 = 9,

    UART_DBUS = 10,
    UART0 = 11,
    UART1 = 12,
    UART2 = 13,
    UART3 = 14,

    IMU = 15,
};

struct CanDataView {
    uint32_t can_id;
    std::span<const std::byte> can_data;
    bool is_fdcan;
    bool is_extended_can_id;
    bool is_remote_transmission;
};

struct UartDataView {
    std::span<const std::byte> uart_data;
    bool idle_delimited;
};

struct AccelerometerDataView {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct GyroscopeDataView {
    int16_t x;
    int16_t y;
    int16_t z;
};

class IDataCallback {
public:
    virtual ~IDataCallback() = default;

    // `*_receive_callback` returns `true` if id is valid
    virtual bool can_receive_callback(DataId id, const CanDataView& data) = 0;

    virtual bool uart_receive_callback(DataId id, const UartDataView& data) = 0;

    virtual void accelerometer_receive_callback(const AccelerometerDataView& data) = 0;

    virtual void gyroscope_receive_callback(const GyroscopeDataView& data) = 0;
};

}; // namespace librmcs::data
