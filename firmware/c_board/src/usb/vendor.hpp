#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <usbd_def.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/src/usb/interrupt_safe_buffer.hpp"
#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

extern "C" {
// NOLINTNEXTLINE(readability-identifier-naming)
extern USBD_HandleTypeDef hUsbDeviceFS;
}

class Vendor
    : private core::protocol::DeserializeCallback
    , private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    Vendor() = default;

    static constexpr size_t kMaxPacketSize = 64;

    core::protocol::Serializer& serializer() { return serializer_; }

    void handle_downlink(std::span<const std::byte> buffer, bool finished);

    bool try_transmit();

private:
    static bool device_ready();

    void can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override;

    void uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override;

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override;

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override;

    void error_callback() override;

    core::protocol::Deserializer deserializer_{*this};

    InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};

    const InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
    inline static uint8_t zlp_dummy_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
