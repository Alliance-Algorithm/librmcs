#include "uart.hpp"

#include <cstdint>

#include <usart.h>

#include "core/include/librmcs/data/datas.hpp"
#include "firmware/c_board/src/usb/helper.hpp"

namespace librmcs::firmware::uart {

// NOLINTNEXTLINE(readability-inconsistent-declaration-parameter-name)
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* hal_uart_handle, uint16_t size) {
    Uart* uart;
    data::DataId field_id;

    if (hal_uart_handle == &huart1) {
        uart = uart2.get();
        field_id = data::DataId::kUart2;
    } else if (hal_uart_handle == &huart3) {
        uart = uart_dbus.get();
        field_id = data::DataId::kUartDbus;
    } else if (hal_uart_handle == &huart6) {
        uart = uart1.get();
        field_id = data::DataId::kUart1;
    } else {
        return;
    }

    const bool idle_delimited = HAL_UARTEx_GetRxEventType(hal_uart_handle) == HAL_UART_RXEVENT_IDLE;
    uart->handle_uplink(field_id, usb::get_serializer(), size, idle_delimited);
}

} // namespace librmcs::firmware::uart
