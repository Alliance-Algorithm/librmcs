#include "firmware/c_board/src/can/can.hpp"

#include <can.h>

#include "core/include/librmcs/data/datas.hpp"
#include "firmware/c_board/src/usb/helper.hpp"

namespace librmcs::firmware::can {

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    Can* can;
    data::DataId field_id;

    if (hcan == &hcan1) {
        can = can1.get();
        field_id = data::DataId::kCan1;
    } else if (hcan == &hcan2) {
        can = can2.get();
        field_id = data::DataId::kCan2;
    } else {
        return;
    }

    can->handle_uplink(field_id, usb::get_serializer());
}

} // namespace librmcs::firmware::can
