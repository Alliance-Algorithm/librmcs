#include "firmware/rmcs_board/app/src/can/can.hpp"

#include <cstddef>

#include <hpm_interrupt.h>

#include <librmcs/data/datas.hpp>

#include "board_app.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/peripherals.hpp"

namespace librmcs::firmware::board {

void can_irq_handler(size_t board_can_index) {
    core::utility::assert_debug(board_can_index < can::kCanCount);

    can::can_array[board_can_index]->irq_handler();
}

SDK_DECLARE_EXT_ISR_M(BOARD_CAN0(IRQn_MCAN, ), can0_isr)
void can0_isr() { can_irq_handler(0); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN1(IRQn_MCAN, ), can1_isr)
void can1_isr() { can_irq_handler(1); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN2(IRQn_MCAN, ), can2_isr)
void can2_isr() { can_irq_handler(2); }

SDK_DECLARE_EXT_ISR_M(BOARD_CAN3(IRQn_MCAN, ), can3_isr)
void can3_isr() { can_irq_handler(3); }

} // namespace librmcs::firmware::board

namespace librmcs::firmware {

bool Peripherals::write_can(data::DataId id, const data::CanDataView& data) {
    switch (id) {
    case data::DataId::kCan0: can::can_array[0]->handle_downlink(data); return true;
    case data::DataId::kCan1: can::can_array[1]->handle_downlink(data); return true;
    case data::DataId::kCan2: can::can_array[2]->handle_downlink(data); return true;
    case data::DataId::kCan3: can::can_array[3]->handle_downlink(data); return true;
    default: return false;
    }
}

} // namespace librmcs::firmware
