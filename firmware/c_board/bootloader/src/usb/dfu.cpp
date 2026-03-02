#include "firmware/c_board/bootloader/src/usb/dfu.hpp"

#include <cstdint>

#include <class/dfu/dfu_device.h>

namespace librmcs::firmware::usb {

extern "C" {

uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state) {
    return Dfu::instance().get_timeout_ms(alt, state);
}

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length) {
    const uint8_t status = Dfu::instance().download(alt, block_num, data, length);
    tud_dfu_finish_flashing(status);
}

void tud_dfu_manifest_cb(uint8_t alt) {
    const uint8_t status = Dfu::instance().manifest(alt);
    tud_dfu_finish_flashing(status);
}

uint16_t tud_dfu_upload_cb(uint8_t, uint16_t, uint8_t*, uint16_t) { return 0; }

void tud_dfu_detach_cb() { Dfu::instance().detach(); }

void tud_dfu_abort_cb(uint8_t alt) { Dfu::instance().abort(alt); }

} // extern "C"

} // namespace librmcs::firmware::usb
