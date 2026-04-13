#include "firmware/rmcs_board/app/src/uart/uart.hpp"

#include <cstddef>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"

namespace librmcs::firmware::board {

void uart_irq_handler(size_t board_uart_index) {
    core::utility::assert_debug(board_uart_index < uart::kUartCount);

    uart::uart_array[board_uart_index]->irq_handler();
}

void uart_dbus_irq_handler() { uart::uart_dbus->irq_handler(); }

} // namespace librmcs::firmware::board
