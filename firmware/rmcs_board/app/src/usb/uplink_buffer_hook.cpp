#include "firmware/rmcs_board/app/src/usb/interrupt_safe_buffer.hpp"

extern "C" [[gnu::weak]] void librmcs_usb_uplink_buffer_full_hook() noexcept {}
