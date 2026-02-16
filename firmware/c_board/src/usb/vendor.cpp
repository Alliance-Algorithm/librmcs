#include "firmware/c_board/src/usb/vendor.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include <usbd_cdc.h>
#include <usbd_def.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/c_board/src/can/can.hpp"
#include "firmware/c_board/src/uart/uart.hpp"
#include "firmware/c_board/src/usb/helper.hpp"

namespace librmcs::firmware::usb {

core::protocol::Serializer& get_serializer() { return vendor->serializer(); }

void Vendor::handle_downlink(std::span<const std::byte> buffer, bool finished) {
    deserializer_.feed(buffer);
    if (finished)
        deserializer_.finish_transfer();
}

void Vendor::can_deserialized_callback(core::protocol::FieldId id, const data::CanDataView& data) {
    switch (id) {
    case data::DataId::kCan1: can::can1->handle_downlink(data); break;
    case data::DataId::kCan2: can::can2->handle_downlink(data); break;
    default: core::utility::assert_failed_always();
    }
}

void Vendor::uart_deserialized_callback(
    core::protocol::FieldId id, const data::UartDataView& data) {
    switch (id) {
    case data::DataId::kUartDbus: uart::uart_dbus->handle_downlink(data); break;
    case data::DataId::kUart1: uart::uart1->handle_downlink(data); break;
    case data::DataId::kUart2: uart::uart2->handle_downlink(data); break;
    default: core::utility::assert_failed_always();
    }
}

void Vendor::accelerometer_deserialized_callback(const data::AccelerometerDataView& data) {
    (void)data;
}

void Vendor::gyroscope_deserialized_callback(const data::GyroscopeDataView& data) { (void)data; }

void Vendor::error_callback() { core::utility::assert_failed_always(); }

bool Vendor::device_ready() {
    auto hal_cdc_handle_atomic =
        std::atomic_ref<void*>(hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId]);
    void* hal_cdc_handle = hal_cdc_handle_atomic.load(std::memory_order_relaxed);

    if (!hal_cdc_handle)
        return false;

    return static_cast<USBD_CDC_HandleTypeDef*>(hal_cdc_handle)->TxState == 0U;
}

bool Vendor::try_transmit() {
    if (!device_ready())
        return false;

    if (!transmitting_batch_)
        transmitting_batch_ = transmit_buffer_.pop_batch();
    if (!transmitting_batch_)
        return false;

    const auto data = transmitting_batch_->data();

    const auto target_size = std::min(data.size() - transmitted_size_, kMaxPacketSize);
    uint8_t* src = nullptr;
    uint16_t len = 0;

    if (target_size) {
        // C HAL API requires non-const access.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        src = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>( //
                data.data() + transmitted_size_));
        len = static_cast<uint16_t>(target_size);
    } else {
        src = &zlp_dummy_;
        len = 0;
    }

    core::utility::assert_always(
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, src, len) == USBD_OK
        && USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK);

    transmitted_size_ += target_size;
    if (transmitted_size_ == data.size() && target_size < kMaxPacketSize) {
        transmit_buffer_.release_batch(transmitting_batch_);
        transmitting_batch_ = nullptr;
        transmitted_size_ = 0;
    }

    return true;
}

namespace {

alignas(size_t) uint8_t receive_buffer[64];

int8_t hal_cdc_init_callback() {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, receive_buffer);
    return USBD_OK;
}

int8_t hal_cdc_deinit_callback() { return USBD_OK; }

// NOLINTNEXTLINE(readability-non-const-parameter) because bullshit HAL api.
int8_t hal_cdc_control_callback(uint8_t command, uint8_t* buffer, uint16_t length) {
    (void)command;
    (void)buffer;
    (void)length;
    return USBD_OK;
}

// NOLINTNEXTLINE(readability-non-const-parameter)
int8_t hal_cdc_receive_callback(uint8_t* buffer, uint32_t* length) {
    const auto size = static_cast<size_t>(*length);

    vendor->handle_downlink(
        {reinterpret_cast<const std::byte*>(buffer), size}, size < Vendor::kMaxPacketSize);

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, buffer);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

// NOLINTNEXTLINE(readability-non-const-parameter)
int8_t hal_cdc_transmit_complete_callback(uint8_t* buffer, uint32_t* length, uint8_t endpoint_num) {
    (void)buffer;
    (void)length;
    (void)endpoint_num;
    return USBD_OK;
}

} // namespace

extern "C" {

// NOLINTNEXTLINE(readability-identifier-naming)
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    .Init = hal_cdc_init_callback,
    .DeInit = hal_cdc_deinit_callback,
    .Control = hal_cdc_control_callback,
    .Receive = hal_cdc_receive_callback,
    .TransmitCplt = hal_cdc_transmit_complete_callback,
};

} // extern "C"

} // namespace librmcs::firmware::usb
