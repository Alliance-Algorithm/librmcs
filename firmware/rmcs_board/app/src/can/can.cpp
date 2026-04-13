#include "firmware/rmcs_board/app/src/can/can.hpp"

#include <cstddef>

#include "core/src/utility/assert.hpp"

namespace librmcs::firmware::board {

void can_irq_handler(size_t board_can_index) {
    core::utility::assert_debug(board_can_index < can::kCanCount);

    can::can_array[board_can_index]->irq_handler();
}

} // namespace librmcs::firmware::board
