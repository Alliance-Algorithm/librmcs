#include "firmware/rmcs_board/src/can/can.hpp"

#include <cstdint>

#include <hpm_mcan_drv.h>
#include <hpm_mcan_regs.h>
#include <hpm_soc.h>
#include <hpm_soc_irq.h>

#include "core/include/librmcs/data/datas.hpp"
#include "firmware/rmcs_board/src/usb/vendor.hpp"

namespace librmcs::firmware::can {

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, can0_isr)
void can0_isr() {
    MCAN_Type* base = HPM_MCAN0;
    const uint32_t flags = mcan_get_interrupt_flags(base);

    if (!flags) [[unlikely]]
        return;

    if (flags & MCAN_INT_RXFIFO0_NEW_MSG) [[likely]]
        can0->handle_uplink(data::DataId::kCan0, usb::vendor->serializer());

    mcan_clear_interrupt_flags(base, flags);
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN1, can1_isr)
void can1_isr() {
    MCAN_Type* base = HPM_MCAN1;
    const uint32_t flags = mcan_get_interrupt_flags(base);

    if (!flags) [[unlikely]]
        return;

    if (flags & MCAN_INT_RXFIFO0_NEW_MSG) [[likely]]
        can1->handle_uplink(data::DataId::kCan1, usb::vendor->serializer());

    mcan_clear_interrupt_flags(base, flags);
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN2, can2_isr)
void can2_isr() {
    MCAN_Type* base = HPM_MCAN2;
    const uint32_t flags = mcan_get_interrupt_flags(base);

    if (!flags) [[unlikely]]
        return;

    if (flags & MCAN_INT_RXFIFO0_NEW_MSG) [[likely]]
        can2->handle_uplink(data::DataId::kCan2, usb::vendor->serializer());

    mcan_clear_interrupt_flags(base, flags);
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, can3_isr)
void can3_isr() {
    MCAN_Type* base = HPM_MCAN3;
    const uint32_t flags = mcan_get_interrupt_flags(base);

    if (!flags) [[unlikely]]
        return;

    if (flags & MCAN_INT_RXFIFO0_NEW_MSG) [[likely]]
        can3->handle_uplink(data::DataId::kCan3, usb::vendor->serializer());

    mcan_clear_interrupt_flags(base, flags);
}

} // namespace librmcs::firmware::can
