#include "firmware/rmcs_board/bootloader/src/usb/dfu.hpp"

#include <cstdint>

#include <class/dfu/dfu_device.h>

namespace librmcs::firmware::usb {

extern "C" {

uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state) {
    return Dfu::get_timeout_ms(alt, state);
}

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length) {
    tud_dfu_finish_flashing(Dfu::instance().download(alt, block_num, data, length));
}

void tud_dfu_manifest_cb(uint8_t alt) { tud_dfu_finish_flashing(Dfu::instance().manifest(alt)); }

uint16_t tud_dfu_upload_cb(uint8_t, uint16_t, uint8_t*, uint16_t) { return 0; }

void tud_dfu_detach_cb() { Dfu::detach(); }

void tud_dfu_abort_cb(uint8_t alt) { Dfu::instance().abort(alt); }

} // extern "C"

} // namespace librmcs::firmware::usb
