#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <main.h>
#include <usart.h>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/c_board/src/led/led.hpp"
#include "firmware/c_board/src/utility/lazy.hpp"

namespace librmcs::firmware::uart {

class Uart {
public:
    using Lazy = utility::Lazy<Uart, UART_HandleTypeDef*, size_t>;

    explicit Uart(UART_HandleTypeDef* hal_uart_handle, uint16_t max_receive_size)
        : hal_uart_handle_(hal_uart_handle)
        , max_receive_size_(max_receive_size) {
        core::utility::assert_always(max_receive_size_ <= 64);
        core::utility::assert_always(trigger_hal_receive());
    }

    void handle_downlink(const data::UartDataView& data) {
        const size_t size = data.uart_data.size();
        if (!size)
            return;

        auto& transmit_buffer = transmit_buffers_[buffer_writing_.load(std::memory_order::relaxed)];
        const uint8_t written_size = transmit_buffer.written_size.load(std::memory_order::relaxed);

        size_t size_allowed = size;
        const auto remaining_size = sizeof(transmit_buffer.data) - written_size;
        size_allowed = std::min(size_allowed, remaining_size);

        transmit_buffer.written_size.store(
            static_cast<uint8_t>(written_size + size_allowed), std::memory_order::relaxed);
        std::memcpy(&transmit_buffer.data[written_size], data.uart_data.data(), size_allowed);

        if (size_allowed != size) [[unlikely]]
            led::led->downlink_buffer_full();
    }

    bool try_transmit() {
        // Under normal circumstances, the trigger_hal_receive function is called within the
        // interrupt service routine (ISR). However, if the ISR fails to execute for any reason, the
        // UART will stop receiving data. Therefore, it is necessary to compensate for this
        // situation by using polling. If the UART is idle, receiving should be triggered again.
        if (device_reception_ready()) [[unlikely]]
            trigger_hal_receive();

        auto writing = buffer_writing_.load(std::memory_order::relaxed);
        if (transmit_buffers_[writing].written_size.load(std::memory_order::relaxed) == 0)
            return false;

        if (!device_transmission_ready())
            return false;

        transmit_buffers_[!writing].written_size.store(0, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order::release);
        buffer_writing_.store(!writing, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order::release);

        // Note: Must read written_size again here to avoid data loss.
        core::utility::assert_always(
            HAL_UART_Transmit_IT(
                hal_uart_handle_, reinterpret_cast<uint8_t*>(transmit_buffers_[writing].data),
                transmit_buffers_[writing].written_size.load(std::memory_order::relaxed))
            == HAL_OK);

        return true;
    }

    void handle_uplink(
        data::DataId field_id, core::protocol::Serializer& serializer, uint16_t size,
        bool is_idle) {
        if (!size) {
            core::utility::assert_always(trigger_hal_receive());
            return;
        }

        core::utility::assert_debug(
            serializer.write_uart(
                field_id,
                {
                    .uart_data = {receive_buffer_, size},
                    .idle_delimited = is_idle,
        })
            != core::protocol::Serializer::SerializeResult::kInvalidArgument);

        core::utility::assert_always(trigger_hal_receive());
    }

private:
    bool device_transmission_ready() const {
        return hal_uart_handle_->gState == HAL_UART_STATE_READY;
    }
    bool device_reception_ready() const {
        return hal_uart_handle_->RxState == HAL_UART_STATE_READY;
    }

    bool trigger_hal_receive() {
        return HAL_UARTEx_ReceiveToIdle_IT(
                   hal_uart_handle_, reinterpret_cast<uint8_t*>(receive_buffer_), max_receive_size_)
            == HAL_OK;
    }

    UART_HandleTypeDef* hal_uart_handle_;

    std::byte receive_buffer_[64];
    uint16_t max_receive_size_;

    struct {
        std::atomic<uint8_t> written_size = 0;
        std::byte data[128];
    } transmit_buffers_[2];
    std::atomic<uint8_t> buffer_writing_ = 0;
};

inline constinit Uart::Lazy uart1{&huart6, 15};
inline constinit Uart::Lazy uart2{&huart1, 15};
inline constinit Uart::Lazy uart_dbus{&huart3, 31};

} // namespace librmcs::firmware::uart
