#include "firmware/rmcs_board/app/src/gpio/gpio.hpp"

#include <cstdint>

#include "board_app.hpp"

namespace librmcs::firmware::board {

void gpio_irq_handler(uint32_t port_index) { gpio::gpio->handle_port_interrupt(port_index); }

} // namespace librmcs::firmware::board
